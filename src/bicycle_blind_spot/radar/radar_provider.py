"""
Radar provider — TI mmWave point-cloud tracker.

Ports the proven Radar_Tracker_Filter_Outdoor_v2.py pipeline into a
reusable provider class:

  Serial data → TLV parse → ROI filter → DBSCAN cluster →
  vehicle-like filter → Kalman track → best-target select → RadarResult

Two range presets are available:

  "standard"  — slope 40 MHz/μs, max range ~27 m  (original config)
  "long"      — slope 12 MHz/μs, max range ~90 m  (for 80-100 m goal)

The long-range preset changes only profileCfg slope and CFAR FOV range.
Everything else (frame rate, chirps, Doppler window) stays the same.

Usage:
    provider = RadarProvider(cfg_port="/dev/ttyUSB0", data_port="/dev/ttyUSB1")
    provider.configure()          # sends commands to sensor
    while True:
        result = provider.read()  # blocks until next radar frame
"""

from __future__ import annotations

import queue
import struct
import time
import os
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import serial
from sklearn.cluster import DBSCAN


# ------------------------------------------------------------------ #
# Packet sync
# ------------------------------------------------------------------ #

MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"


# ------------------------------------------------------------------ #
# Sensor command sets
# ------------------------------------------------------------------ #

# Shared commands (everything except profileCfg and cfarFovCfg range)
_CMDS_COMMON_HEAD = [
    "sensorStop",
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 15 5 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "lowPower 0 0",
]

_CMDS_COMMON_TAIL = [
    "chirpCfg 0 0 0 0 0 0 0 1",
    "chirpCfg 1 1 0 0 0 0 0 4",
    "frameCfg 0 1 32 0 100 1 0",              # 10 Hz
    "guiMonitor -1 1 1 1 0 0 1",
    "cfarCfg -1 0 2 8 4 3 0 15.0 0",          # range CFAR
    "cfarCfg -1 1 0 4 2 3 1 15.0 0",          # Doppler CFAR
    "multiObjBeamForming -1 1 0.5",
    "calibDcRangeSig -1 0 -5 8 256",
    "clutterRemoval -1 1",                     # outdoor: remove static
    "compRangeBiasAndRxChanPhase 0.0 "
        "1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0",
    "measureRangeBiasAndRxChanPhase 0 1. 0.2",
    "aoaFovCfg -1 -90 90 -90 90",
]

_CMDS_COMMON_BOTTOM = [
    "extendedMaxVelocity -1 0",
    "CQRxSatMonitor 0 3 11 121 0",
    "CQSigImgMonitor 0 127 8",
    "analogMonitor 0 0",
    "lvdsStreamCfg -1 0 0 0",
    "calibData 0 0 0",
    "sensorStart",
]

# Standard range (~27 m): slope 40 MHz/μs, 256 ADC samples @ 7200 ksps
# R_max = c * Fs / (2 * S) = 3e8 * 7.2e6 / (2 * 40e12) ≈ 27 m
CMDS_STANDARD = (
    _CMDS_COMMON_HEAD
    + ["profileCfg 0 77 7 3 39 0 0 40 1 256 7200 0 0 30"]
    + _CMDS_COMMON_TAIL
    + ["cfarFovCfg -1 0 0.25 60.0",
       "cfarFovCfg -1 1 -12.0 12.0"]
    + _CMDS_COMMON_BOTTOM
)

# Long range (~90 m): slope 12 MHz/μs, longer ramp, same ADC
# R_max = 3e8 * 7.2e6 / (2 * 12e12) ≈ 90 m
# Range resolution = c / (2 * B) where B = 12 MHz/μs * 60 μs = 720 MHz → 0.21 m
CMDS_LONG_RANGE = (
    _CMDS_COMMON_HEAD
    + ["profileCfg 0 77 7 3 60 0 0 12 1 256 7200 0 0 30"]
    + _CMDS_COMMON_TAIL
    + ["cfarFovCfg -1 0 0.25 100.0",
       "cfarFovCfg -1 1 -15.0 15.0"]
    + _CMDS_COMMON_BOTTOM
)

RANGE_PRESETS = {
    "standard": CMDS_STANDARD,
    "long": CMDS_LONG_RANGE,
}


# ------------------------------------------------------------------ #
# TLV parsing
# ------------------------------------------------------------------ #

def _parse_tlv1_points(payload: bytes) -> Optional[np.ndarray]:
    """Parse TLV type 1 (point cloud) → Nx4 float32 [x, y, z, v_r]."""
    L = len(payload)
    for trim in (0, 8, 4, 12):
        if L > trim and ((L - trim) % 16) == 0:
            body = payload[:L - trim]
            pts = np.frombuffer(body, dtype=np.float32)
            if pts.size >= 4 and (pts.size % 4) == 0:
                pts = pts.reshape(-1, 4)
                pts = pts[np.isfinite(pts).all(axis=1)]
                return pts
    return None


# ------------------------------------------------------------------ #
# Clustering
# ------------------------------------------------------------------ #

@dataclass
class Cluster:
    cx: float
    cy: float
    cz: float
    v_r_med: float
    v_r_std: float
    n: int
    extent_x: float
    extent_y: float


def _cluster_dbscan(
    pts: np.ndarray, eps_m: float, min_samples: int,
) -> List[Cluster]:
    if pts.shape[0] < min_samples:
        return []
    labels = DBSCAN(eps=eps_m, min_samples=min_samples).fit_predict(pts[:, :2])
    out: List[Cluster] = []
    for lab in set(int(l) for l in labels.tolist()):
        if lab == -1:
            continue
        m = labels == lab
        clu = pts[m]
        x, y, z, vr = clu[:, 0], clu[:, 1], clu[:, 2], clu[:, 3]
        out.append(Cluster(
            cx=float(np.median(x)),
            cy=float(np.median(y)),
            cz=float(np.median(z)),
            v_r_med=float(np.median(vr)),
            v_r_std=float(np.std(vr)),
            n=int(clu.shape[0]),
            extent_x=float(np.percentile(x, 90) - np.percentile(x, 10)),
            extent_y=float(np.percentile(y, 90) - np.percentile(y, 10)),
        ))
    return out


def _vehicle_like(c: Cluster, min_pts: int = 5) -> bool:
    if c.n < min_pts:
        return False
    if not (0.15 <= c.extent_x <= 5.5):
        return False
    if not (0.15 <= c.extent_y <= 10.0):
        return False
    if c.v_r_std > 3.0:
        return False
    return True


# ------------------------------------------------------------------ #
# Kalman tracker
# ------------------------------------------------------------------ #

@dataclass
class Track:
    tid: int
    x: np.ndarray          # state [x, y, vx, vy]
    P: np.ndarray          # covariance 4×4
    age: int = 0
    hits: int = 0
    misses: int = 0
    last_vr: float = 0.0
    last_n: int = 0
    range_m: float = 0.0
    range_rate_mps: float = 0.0


class _MultiTracker:
    def __init__(
        self,
        dt: float = 0.10,
        gate_m: float = 3.5,
        max_misses: int = 10,
        min_hits_to_confirm: int = 3,
        meas_std_m: float = 0.5,
        accel_std: float = 3.0,
    ):
        self.dt = dt
        self.gate_m = gate_m
        self.max_misses = max_misses
        self.min_hits_to_confirm = min_hits_to_confirm
        self.next_id = 1
        self.tracks: List[Track] = []

        self.R = np.diag([meas_std_m**2, meas_std_m**2]).astype(np.float32)
        self.accel_std = accel_std
        self.F = self._F(dt)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
        self.Q = self._Q(dt, accel_std)

    @staticmethod
    def _F(dt: float) -> np.ndarray:
        return np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=np.float32)

    @staticmethod
    def _Q(dt: float, a: float) -> np.ndarray:
        q = a**2
        dt2, dt3, dt4 = dt*dt, dt*dt*dt, dt*dt*dt*dt
        return q * np.array([
            [dt4/4, 0,     dt3/2, 0    ],
            [0,     dt4/4, 0,     dt3/2],
            [dt3/2, 0,     dt2,   0    ],
            [0,     dt3/2, 0,     dt2  ],
        ], dtype=np.float32)

    def set_dt(self, dt: float):
        self.dt = dt
        self.F = self._F(dt)
        self.Q = self._Q(dt, self.accel_std)

    def predict(self):
        for t in self.tracks:
            t.x = self.F @ t.x
            t.P = self.F @ t.P @ self.F.T + self.Q
            t.age += 1

    def update(self, clusters: List[Cluster]):
        used: set = set()

        for t in self.tracks:
            tx, ty = float(t.x[0]), float(t.x[1])
            best_j, best_d = None, 1e9

            for j, c in enumerate(clusters):
                if j in used:
                    continue
                d = float(np.hypot(c.cx - tx, c.cy - ty))
                if d < best_d:
                    best_d = d
                    best_j = j

            if best_j is None or best_d > self.gate_m:
                t.misses += 1
                continue

            c = clusters[best_j]
            used.add(best_j)

            z = np.array([c.cx, c.cy], dtype=np.float32)
            y = z - (self.H @ t.x)
            S = self.H @ t.P @ self.H.T + self.R
            K = t.P @ self.H.T @ np.linalg.inv(S)

            t.x = t.x + K @ y
            t.P = (np.eye(4, dtype=np.float32) - K @ self.H) @ t.P
            t.hits += 1
            t.misses = 0
            t.last_vr = c.v_r_med
            t.last_n = c.n
            new_range = float(np.hypot(float(t.x[0]), float(t.x[1])))
            prev_range = t.range_m or new_range
            range_rate = max(0.0, (prev_range - new_range) / max(self.dt, 1e-3))
            t.range_m = new_range
            t.range_rate_mps = range_rate

        # Birth new tracks from unassociated clusters
        for j, c in enumerate(clusters):
            if j in used:
                continue
            x0 = np.array([c.cx, c.cy, 0.0, 0.0], dtype=np.float32)
            P0 = np.diag([1.5, 1.5, 25.0, 25.0]).astype(np.float32)
            range0 = float(np.hypot(c.cx, c.cy))
            self.tracks.append(Track(
                self.next_id, x0, P0, hits=1,
                last_vr=c.v_r_med, last_n=c.n,
                range_m = range0,
                range_rate_mps = 0.0,
            ))
            self.next_id += 1

        # Prune dead tracks
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

    def confirmed(self) -> List[Track]:
        return [t for t in self.tracks
                if t.hits >= self.min_hits_to_confirm and t.misses <= 1]


# ------------------------------------------------------------------ #
# Best-target selection
# ------------------------------------------------------------------ #

def _select_best(
    tracks: List[Track],
    min_range_m: float,
    approaching_sign: int,
) -> Optional[Track]:
    """
    Approaching-first scoring (from Outdoor_v2).
    approaching_sign: +1 if positive v_r = approaching, -1 if negative.
    """
    best, best_score = None, -1e9

    for t in tracks:
        x, y = float(t.x[0]), float(t.x[1])
        r = float(np.hypot(x, y))
        if r < min_range_m:
            continue

        score = 2.0 * t.hits + 0.8 * float(t.last_n)
        score += 2.0 / max(1.0, r)

        if t.last_vr * approaching_sign > 0.3:
            score += 25.0
        else:
            score -= 10.0

        if score > best_score:
            best_score = score
            best = t

    return best


# ------------------------------------------------------------------ #
# Live visualization (optional, main-thread only via step_viz())
# ------------------------------------------------------------------ #

class _LiveViz:
    """
    Top-down radar scatter plot (port of Radar_Tracker_Filter_Outdoor_v2.LiveViz).

    IMPORTANT: all matplotlib calls happen in the MAIN thread via step_viz().
    The background radar thread only enqueues data; it never calls matplotlib.
    """

    def __init__(self, xmax: float, ymin: float, ymax: float):
        # OpenCV wheels can set Qt plugin env vars (for cv2 HighGUI) that clash
        # with matplotlib's Qt backend in fusion mode.
        qt_env_keys = ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR")
        saved_qt_env = {k: os.environ.get(k) for k in qt_env_keys}
        for k in qt_env_keys:
            os.environ.pop(k, None)
        import matplotlib.pyplot as plt  # deferred — only imported when --show used
        for k, v in saved_qt_env.items():
            if v is not None:
                os.environ[k] = v
        self._plt = plt
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Radar Top-Down View (x lateral, y range)")
        self.ax.set_xlabel("x  lateral (m)")
        self.ax.set_ylabel("y  range (m)")
        self.ax.set_xlim(-xmax, xmax)
        self.ax.set_ylim(ymin, ymax)
        self.ax.grid(True)
        self.scat = self.ax.scatter([], [], s=12)
        self.cluster_scat = self.ax.scatter([], [], s=90, marker="x", c="orange", zorder=3)
        self.cbar = self.fig.colorbar(self.scat, ax=self.ax)
        self.cbar.set_label("radial velocity v_r (m/s)")
        self._track_texts: list = []

    def update(
        self,
        pts: Optional[np.ndarray],
        clusters: Optional[List[Cluster]] = None,
        tracks: Optional[List[Tuple[int, float, float]]] = None,
    ):
        plt = self._plt

        # Remove old track labels
        for t in self._track_texts:
            t.remove()
        self._track_texts = []

        if pts is None or pts.size == 0:
            self.scat.set_offsets(np.zeros((0, 2)))
            self.cluster_scat.set_offsets(np.zeros((0, 2)))
            plt.pause(0.001)
            return

        x, y, v = pts[:, 0], pts[:, 1], pts[:, 3]
        self.scat.set_offsets(np.c_[x, y])
        self.scat.set_array(v)
        vmax = max(1.0, min(float(np.nanmax(np.abs(v))) if v.size else 1.0, 20.0))
        self.scat.set_clim(-vmax, vmax)

        if clusters:
            self.cluster_scat.set_offsets(np.c_[[c.cx for c in clusters],
                                                [c.cy for c in clusters]])
        else:
            self.cluster_scat.set_offsets(np.zeros((0, 2)))

        if tracks:
            for tid, tx, ty in tracks:
                self._track_texts.append(
                    self.ax.text(tx, ty, str(tid), fontsize=10, color="red")
                )

        self.fig.canvas.draw_idle()
        plt.pause(0.001)


# ------------------------------------------------------------------ #
# Result dataclass
# ------------------------------------------------------------------ #

@dataclass
class RadarResult:
    """Output of RadarProvider.read()."""
    status: str          # "OK" or "NO_TARGET"
    range_m: float       # distance to best threat (0 if no target)
    closing_mps: float   # approach speed (positive = approaching; 0 if no target)
    angle_deg: float     # azimuth: negative=left, positive=right
    n_tracks: int        # number of confirmed tracks
    approaching: bool    # True if best threat is actively approaching
    radial_mps: float    # raw signed radial velocity after sign correction (positive = approaching; 0 if no target)
    range_rate_mps: float # range rate, positive when estimated range is decreasing; 0 if no target


# ------------------------------------------------------------------ #
# CLI helpers
# ------------------------------------------------------------------ #

def _is_hard_error(line: str) -> bool:
    l = line.lower()
    return "error" in l or "invalid" in l or "unknown" in l or "not recognized" in l


def _send_cmd(ser: serial.Serial, cmd: str, timeout_s: float) -> bool:
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    ser.flush()

    t0 = time.time()
    saw_any = False
    while time.time() - t0 < timeout_s:
        line = ser.readline()
        if not line:
            continue
        s = line.decode(errors="ignore").strip()
        if not s:
            continue
        saw_any = True
        sl = s.lower()
        if "ignored:" in sl and ("already stopped" in sl or "already started" in sl):
            return True
        if _is_hard_error(s):
            print(f"  [radar-cfg] {s}")
            return False
        if "done" in sl:
            return True

    return cmd == "sensorStop" and saw_any


# ------------------------------------------------------------------ #
# FTDI auto-detection
# ------------------------------------------------------------------ #

_FTDI_VID_PID = "0403:6010"  # FTDI FT2232 dual-port adapter


def detect_radar_ports() -> tuple[str, str]:
    """
    Auto-detect the FTDI dual-port radar adapter.

    Returns (cfg_port, data_port).  The FTDI FT2232 exposes two serial
    interfaces; interface 0 is the CLI/config port, interface 1 is the
    high-speed data port.  The ttyUSB numbers can shuffle on every USB
    re-enumeration, so we resolve them fresh each time.

    Falls back to /dev/ttyUSB0, /dev/ttyUSB1 if detection fails.
    """
    from serial.tools.list_ports import comports

    # Collect all ports belonging to the FTDI dual-port adapter,
    # keyed by their USB interface number (0 = cfg, 1 = data).
    ports: dict[int, str] = {}
    for p in comports():
        if p.vid == 0x0403 and p.pid == 0x6010 and p.location:
            # location looks like "1-2:1.0" — trailing digit is interface number
            try:
                iface = int(p.location.rsplit(".", 1)[-1])
                ports[iface] = p.device
            except (ValueError, IndexError):
                continue

    if 0 in ports and 1 in ports:
        print(f"[radar] Auto-detected cfg={ports[0]}  data={ports[1]}")
        return ports[0], ports[1]

    print("[radar] FTDI auto-detect failed, falling back to /dev/ttyUSB0 + /dev/ttyUSB1")
    return "/dev/ttyUSB0", "/dev/ttyUSB1"


# ------------------------------------------------------------------ #
# Provider
# ------------------------------------------------------------------ #

@dataclass
class RadarConfig:
    """All tunables for the radar pipeline."""
    cfg_port: str = "auto"
    data_port: str = "auto"
    range_preset: str = "long"            # "standard" (~27 m) or "long" (~90 m)

    # ROI
    roi_xmax: float = 3.0                 # lateral half-width (m)
    roi_ymin: float = 1.5                 # min range (m)
    roi_ymax: float = 100.0               # max range (m)
    roi_zmin: float = -2.5
    roi_zmax: float = 2.5

    # DBSCAN
    db_eps: float = 0.9
    db_min_samples: int = 4

    # Tracker
    gate_m: float = 3.5
    max_misses: int = 10
    min_hits_to_confirm: int = 3

    # Selection
    min_select_range: float = 1.5
    approaching_sign: int = 1             # +1 or -1, calibrate on first test

    # Visualization (main-thread only via step_viz())
    show: bool = False
    verbose: bool = False # print per-frame RADAR summaries

PRINT_EVERY_SEC = 1.0


class RadarProvider:
    """
    TI mmWave radar provider.

    configure() sends commands to the CLI port to start the sensor.
    read() blocks until the next radar frame, processes the point cloud
    through the full pipeline, and returns a RadarResult.
    """

    def __init__(self, cfg: Optional[RadarConfig] = None):
        self.cfg = cfg or RadarConfig()
        self._auto = self.cfg.cfg_port == "auto" or self.cfg.data_port == "auto"

        self._tracker = _MultiTracker(
            dt=0.10,
            gate_m=self.cfg.gate_m,
            max_misses=self.cfg.max_misses,
            min_hits_to_confirm=self.cfg.min_hits_to_confirm,
        )

        self._ser: Optional[serial.Serial] = None
        self._buf = bytearray()
        self._t_last = time.time()
        self._last_print = 0.0

        # Viz: created lazily in main thread by first call to step_viz()
        self._viz: Optional[_LiveViz] = None
        # Background thread enqueues (pts_roi, clusters, tracks); main thread drains
        self._viz_q: Optional[queue.Queue] = (
            queue.Queue(maxsize=2) if self.cfg.show else None
        )

    def _resolve_ports(self):
        """Resolve 'auto' port values, always re-detecting (ports can shift
        after USB re-enumeration triggered by sensorStart)."""
        if self._auto:
            cfg_port, data_port = detect_radar_ports()
            self.cfg.cfg_port = cfg_port
            self.cfg.data_port = data_port

    def configure(self):
        """Send config commands to sensor CLI port."""
        self._resolve_ports()
        cmds = RANGE_PRESETS.get(self.cfg.range_preset, CMDS_LONG_RANGE)
        port = self.cfg.cfg_port

        print(f"[radar] Configuring sensor on {port} @ 115200 "
              f"(preset={self.cfg.range_preset})")
        with serial.Serial(port, 115200, timeout=0.2, write_timeout=1) as s:
            time.sleep(0.35)
            s.reset_input_buffer()
            _ = s.read(4096)
            for cmd in cmds:
                to = 4.0 if cmd == "sensorStart" else 1.6
                if not _send_cmd(s, cmd, to):
                    raise RuntimeError(f"[radar] Command failed: {cmd}")
                time.sleep(0.05)
        print("[radar] Sensor started.")

    def start_data(self):
        """Open the data serial port."""
        # sensorStart can trigger a USB re-enumeration, shifting ttyUSB
        # numbers.  Wait briefly for the device to settle, then re-detect.
        if self._auto:
            time.sleep(0.6)
            self._resolve_ports()
        self._ser = serial.Serial(self.cfg.data_port, 921600, timeout=0.05)
        self._buf.clear()
        self._t_last = time.time()

    def reconnect_data(self):
        """
        Re-open the data port after a USB re-enumeration (EMI glitch).

        The radar sensor keeps running — only the USB serial link dropped.
        Close the dead handle, wait for the kernel to re-enumerate the FTDI
        device, re-detect the port number, and re-open.
        """
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        # Wait for USB to settle after re-enumeration (~500ms from dmesg)
        time.sleep(1.0)

        if self._auto:
            self._resolve_ports()

        print(f"[radar] Reconnecting data port: {self.cfg.data_port}")
        self._ser = serial.Serial(self.cfg.data_port, 921600, timeout=0.05)
        self._buf.clear()
        self._t_last = time.time()
        print(f"[radar] Data port reconnected.")

    def stop(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            self._ser = None

    def read(self) -> RadarResult:
        """
        Block until the next complete radar frame, run the full pipeline,
        return a RadarResult.
        """
        assert self._ser is not None, "call start_data() first"

        while True:
            chunk = self._ser.read(4096)
            now = time.time()

            if not chunk:
                time.sleep(0.002)
                continue

            self._buf.extend(chunk)

            # Find magic header
            mi = self._buf.find(MAGIC)
            if mi == -1:
                if len(self._buf) > 50_000:
                    del self._buf[:-20_000]
                continue
            if len(self._buf) < mi + 40:
                continue

            header = self._buf[mi:mi + 40]
            total_len = struct.unpack_from("<I", header, 12)[0]
            if total_len < 40 or total_len > 20_000:
                del self._buf[:mi + 8]
                continue
            if len(self._buf) < mi + total_len:
                continue

            # Complete packet
            pkt = bytes(self._buf[mi:mi + total_len])
            del self._buf[:mi + total_len]

            dt = max(0.02, min(0.25, now - self._t_last))
            self._t_last = now
            self._tracker.set_dt(dt)

            # Parse TLVs
            pts_roi = self._parse_packet(pkt)

            # Pipeline: predict → cluster → filter → update → select
            self._tracker.predict()

            clusters: List[Cluster] = []
            if pts_roi is not None and pts_roi.shape[0] > 0:
                clusters = _cluster_dbscan(
                    pts_roi, self.cfg.db_eps, self.cfg.db_min_samples,
                )
                clusters = [c for c in clusters if _vehicle_like(c)]

            self._tracker.update(clusters)

            conf = self._tracker.confirmed()
            best = _select_best(
                conf,
                min_range_m=self.cfg.min_select_range,
                approaching_sign=self.cfg.approaching_sign,
            )

            result = self._build_result(best)

            # Enqueue viz data for the main thread (non-blocking; drops frame if full)
            if self._viz_q is not None:
                tracks_for_viz = [
                    (t.tid, float(t.x[0]), float(t.x[1])) for t in conf
                ]
                try:
                    self._viz_q.put_nowait((pts_roi, clusters, tracks_for_viz))
                except queue.Full:
                    pass

            # Throttled console print
            if self.cfg.verbose and now - self._last_print >= PRINT_EVERY_SEC:
                self._last_print = now
                r = result
                if r.status != "NO_TARGET":
                    print(
                        f"[radar] range={r.range_m:5.1f}m "
                        f"close={r.closing_mps:+5.2f}m/s "
                        f"vr={r.radial_mps:+5.2f}m/s "
                        f"rr={r.range_rate_mps:+5.2f}m/s "
                        f"az={r.angle_deg:+5.1f}° "
                        f"tracks={r.n_tracks}"
                    )
                else:
                    npts = 0 if pts_roi is None else int(pts_roi.shape[0])
                    print(
                        f"[radar] NO_TARGET | roi_pts={npts} "
                        f"tracks={len(self._tracker.tracks)}"
                    )

            return result

    def step_viz(self):
        """
        Render the latest radar frame in the live plot.

        MUST be called from the main thread (matplotlib requirement).
        Call in a tight loop (~20 Hz) when --show is enabled.
        No-op if show=False.
        """
        if self._viz_q is None:
            return

        # Create the plot window the first time (must be on main thread)
        if self._viz is None:
            self._viz = _LiveViz(
                xmax=self.cfg.roi_xmax,
                ymin=0.0,
                ymax=self.cfg.roi_ymax,
            )

        # Drain and render the newest queued frame
        frame = None
        while True:
            try:
                frame = self._viz_q.get_nowait()
            except queue.Empty:
                break

        if frame is not None:
            pts_roi, clusters, tracks_for_viz = frame
            self._viz.update(pts_roi, clusters=clusters, tracks=tracks_for_viz)

    def _parse_packet(self, pkt: bytes) -> Optional[np.ndarray]:
        """Extract ROI-filtered point cloud from a TLV packet."""
        num_tlvs = struct.unpack_from("<I", pkt, 28)[0]
        offset = 40
        pts_roi = None

        for _ in range(num_tlvs):
            if offset + 8 > len(pkt):
                break
            tlv_type, tlv_len = struct.unpack_from("<II", pkt, offset)
            offset += 8
            if tlv_len < 8 or offset + (tlv_len - 8) > len(pkt):
                break

            payload = pkt[offset:offset + tlv_len - 8]

            if tlv_type == 1:
                pts = _parse_tlv1_points(payload)
                if pts is not None and pts.size > 0:
                    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
                    m = (
                        (np.abs(x) < self.cfg.roi_xmax)
                        & (y > self.cfg.roi_ymin) & (y < self.cfg.roi_ymax)
                        & (z > self.cfg.roi_zmin) & (z < self.cfg.roi_zmax)
                    )
                    pts_roi = pts[m]

            offset += (tlv_len - 8)

        return pts_roi

    def _build_result(self, best: Optional[Track]) -> RadarResult:
        if best is None:
            return RadarResult(
                status="NO_TARGET", range_m=0.0, closing_mps=0.0,
                angle_deg=0.0, n_tracks=0, approaching=False,
                radial_mps=0.0, range_rate_mps=0.0,
            )

        x, y = float(best.x[0]), float(best.x[1])
        range_m = float(np.hypot(x, y))
        angle_deg = float(np.degrees(np.arctan2(x, y)))
        radial_mps = best.last_vr * self.cfg.approaching_sign
        radial_closing = max(0.0, radial_mps)  # only count positive (approaching) radial velocity
        if radial_closing > 0.0 and best.range_rate_mps > 0.0:
            closing_raw = 0.65 * radial_closing + 0.35 * best.range_rate_mps # blend raw radial velocity with range rate for more stable closing speed estimate
        else:
            closing_raw = max(radial_closing, best.range_rate_mps)  # if not approaching, take the max of radial and range rate to avoid underestimating closing speed

            

        return RadarResult(
            status="OK",
            range_m=range_m,
            closing_mps=closing_raw,
            angle_deg=angle_deg,
            n_tracks=len(self._tracker.confirmed()),
            approaching=closing_raw > 0.3,
            radial_mps=radial_mps,
            range_rate_mps=best.range_rate_mps,
        )
