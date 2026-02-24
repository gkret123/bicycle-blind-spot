"""
Bike rear-radar point-cloud tracker (no ML)

Window-testing tuned version:
- More tolerant clustering/tracking to reduce flicker on sparse/static scenes.
- We'll later add TTC + approaching-only + lane corridor filters.
"""

import time
import struct
import argparse
from dataclasses import dataclass
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import serial
import numpy as np
from sklearn.cluster import DBSCAN

# -----------------------------
# Radar config + packet constants
# -----------------------------

MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"

CMDS = [
    "sensorStop",
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 15 5 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "lowPower 0 0",
    "profileCfg 0 77 7 3 39 0 0 100 1 256 7200 0 0 30",
    "chirpCfg 0 0 0 0 0 0 0 1",
    "chirpCfg 1 1 0 0 0 0 0 4",
    "frameCfg 0 1 32 0 100 1 0",
    "guiMonitor -1 1 1 1 0 0 1",  # points ON, heatmaps OFF
    "cfarCfg -1 0 2 8 4 3 0 15.0 0",
    "cfarCfg -1 1 0 4 2 3 1 15.0 0",
    "multiObjBeamForming -1 1 0.5",
    "calibDcRangeSig -1 0 -5 8 256",

    # Window/static testing: OFF so static objects can appear
    "clutterRemoval -1 0",

    "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0",
    "measureRangeBiasAndRxChanPhase 0 1. 0.2",
    "aoaFovCfg -1 -90 90 -90 90",

    # Range + Doppler FOV gating
    "cfarFovCfg -1 0 0.25 60.0",
    "cfarFovCfg -1 1 -10.59 10.59",

    "extendedMaxVelocity -1 0",
    "CQRxSatMonitor 0 3 11 121 0",
    "CQSigImgMonitor 0 127 8",
    "analogMonitor 0 0",
    "lvdsStreamCfg -1 0 0 0",
    "calibData 0 0 0",
    "sensorStart",
]


def is_hard_error(line: str) -> bool:
    l = line.lower()
    return ("error" in l) or ("invalid" in l) or ("unknown" in l) or ("not recognized" in l)


def send_cmd_wait_done(ser: serial.Serial, cmd: str, timeout_s: float) -> bool:
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
        if is_hard_error(s):
            print("  ", s)
            return False
        if "done" in sl:
            return True

    if cmd == "sensorStop" and saw_any:
        return True
    return False


def configure_sensor(cfg_port: str):
    print(f"\n== Configuring sensor on {cfg_port} @ 115200 ==")
    with serial.Serial(cfg_port, 115200, timeout=0.2, write_timeout=1) as s:
        time.sleep(0.35)
        s.reset_input_buffer()
        _ = s.read(4096)
        for c in CMDS:
            to = 4.0 if c == "sensorStart" else 1.6
            if not send_cmd_wait_done(s, c, to):
                raise RuntimeError(f"Command failed: {c}")
            time.sleep(0.05)
    print("✅ Sensor started.")


# -----------------------------
# TLV parsing
# -----------------------------

def parse_tlv1_points(payload: bytes) -> Optional[np.ndarray]:
    # float32 [x,y,z,vel]*N with possible footer bytes
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


def to_rva(x: float, y: float, v_r: float) -> Tuple[float, float, float]:
    rng = float(np.hypot(x, y))
    az = float(np.degrees(np.arctan2(x, y)))  # 0=fwd-ish (depends on frame)
    return rng, az, float(v_r)


# -----------------------------
# Visualization
# -----------------------------

class LiveViz:
    def __init__(self, xmax: float, ymin: float, ymax: float):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Radar Top-Down View (x lateral, y range)")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_xlim(-xmax, xmax)
        self.ax.set_ylim(ymin, ymax)
        self.ax.grid(True)

        self.scat = self.ax.scatter([], [], s=10)
        self.cbar = self.fig.colorbar(self.scat, ax=self.ax)
        self.cbar.set_label("radial velocity v_r (m/s)")

        self.cluster_scat = self.ax.scatter([], [], s=80, marker="x")
        self.track_text = []

    def update(self, pts: Optional[np.ndarray], clusters: Optional[List] = None, tracks: Optional[List] = None):
        for t in self.track_text:
            t.remove()
        self.track_text = []

        if pts is None or pts.size == 0:
            self.scat.set_offsets(np.zeros((0, 2)))
            self.cluster_scat.set_offsets(np.zeros((0, 2)))
            plt.pause(0.001)
            return

        x = pts[:, 0]
        y = pts[:, 1]
        v = pts[:, 3]

        self.scat.set_offsets(np.c_[x, y])
        self.scat.set_array(v)

        if clusters:
            cx = [c.cx for c in clusters]
            cy = [c.cy for c in clusters]
            self.cluster_scat.set_offsets(np.c_[cx, cy])
        else:
            self.cluster_scat.set_offsets(np.zeros((0, 2)))

        if tracks:
            for tid, tx, ty in tracks:
                self.track_text.append(self.ax.text(tx, ty, str(tid), fontsize=10))

        self.fig.canvas.draw_idle()
        plt.pause(0.001)


# -----------------------------
# Clustering + heuristics
# -----------------------------

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
    pts: np.ndarray


def cluster_dbscan(pts_xyv: np.ndarray, eps_m: float, min_samples: int) -> List[Cluster]:
    if pts_xyv.shape[0] < min_samples:
        return []

    X = pts_xyv[:, :2]  # x,y
    labels = DBSCAN(eps=eps_m, min_samples=min_samples).fit_predict(X)

    out: List[Cluster] = []
    for lab in set(int(l) for l in labels.tolist()):
        if lab == -1:
            continue
        m = labels == lab
        clu = pts_xyv[m]
        x, y, z, vr = clu[:, 0], clu[:, 1], clu[:, 2], clu[:, 3]

        cx = float(np.median(x))
        cy = float(np.median(y))
        cz = float(np.median(z))
        vmed = float(np.median(vr))
        vstd = float(np.std(vr))

        ex = float(np.percentile(x, 90) - np.percentile(x, 10))
        ey = float(np.percentile(y, 90) - np.percentile(y, 10))

        out.append(Cluster(cx, cy, cz, vmed, vstd, int(clu.shape[0]), ex, ey, clu))
    return out


def vehicle_like(c: Cluster) -> bool:
    # Window/static scenes often produce fewer detections on a real object
    if c.n < 3:
        return False

    # Keep broad bounds for now
    if not (0.10 <= c.extent_x <= 6.0):
        return False
    if not (0.10 <= c.extent_y <= 15.0):
        return False

    # Coherent-ish Doppler (still noisy through window/edges)
    if c.v_r_std > 4.0:
        return False

    return True


# -----------------------------
# Tracking (classic constant-velocity KF)
# -----------------------------

@dataclass
class Track:
    tid: int
    x: np.ndarray  # [x,y,vx,vy]
    P: np.ndarray
    age: int = 0
    hits: int = 0
    misses: int = 0
    last_vr: float = 0.0


class MultiTracker:
    def __init__(
        self,
        dt: float = 0.10,
        gate_m: float = 4.5,          # looser gate to reduce ID churn
        max_misses: int = 15,         # survive dropouts
        min_hits_to_confirm: int = 2, # faster confirmation for debug
        meas_std_m: float = 0.6,
        accel_std: float = 2.5,
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
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=np.float32)
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
    def _Q(dt: float, accel_std: float) -> np.ndarray:
        q = accel_std**2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        return q * np.array([
            [dt4/4, 0,     dt3/2, 0],
            [0,     dt4/4, 0,     dt3/2],
            [dt3/2, 0,     dt2,   0],
            [0,     dt3/2, 0,     dt2],
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
        used = set()

        # Greedy nearest-neighbor association
        for t in self.tracks:
            tx, ty = float(t.x[0]), float(t.x[1])

            best_j = None
            best_d = 1e9
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

        # Spawn new tracks from unassigned clusters
        for j, c in enumerate(clusters):
            if j in used:
                continue
            x0 = np.array([c.cx, c.cy, 0.0, 0.0], dtype=np.float32)
            P0 = np.diag([2.0, 2.0, 20.0, 20.0]).astype(np.float32)
            self.tracks.append(Track(self.next_id, x0, P0, hits=1, last_vr=c.v_r_med))
            self.next_id += 1

        # Prune dead tracks
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

    def confirmed_tracks(self) -> List[Track]:
        # Allow small misses so "NO_TARGET" doesn't spam on one dropped frame
        return [t for t in self.tracks if t.hits >= self.min_hits_to_confirm and t.misses <= 2]


# -----------------------------
# Streaming loop
# -----------------------------

def run_stream(
    data_port: str,
    fps_hint: float,
    roi_xmax: float,
    roi_ymin: float,
    roi_ymax: float,
    roi_zmin: float,
    roi_zmax: float,
    db_eps: float,
    db_min_samples: int,
    viz: bool,
):
    print(f"== Streaming on {data_port} @ 921600 ==")
    viewer = LiveViz(xmax=roi_xmax, ymin=0.0, ymax=roi_ymax) if viz else None

    buf = bytearray()
    last_print = 0.0

    dt_init = 1.0 / max(1.0, float(fps_hint))
    tracker = MultiTracker(dt=dt_init)  # uses tuned defaults above

    with serial.Serial(data_port, 921600, timeout=0.05) as ser:
        t_last = time.time()

        while True:
            chunk = ser.read(4096)
            now = time.time()

            if not chunk:
                time.sleep(0.002)
                continue

            buf.extend(chunk)

            mi = buf.find(MAGIC)
            if mi == -1:
                if len(buf) > 50000:
                    del buf[:-20000]
                continue
            if len(buf) < mi + 40:
                continue

            header = buf[mi:mi + 40]
            total_len = struct.unpack_from("<I", header, 12)[0]
            if total_len < 40 or total_len > 20000:
                del buf[:mi + 8]
                continue
            if len(buf) < mi + total_len:
                continue

            pkt = bytes(buf[mi:mi + total_len])
            del buf[:mi + total_len]

            # Update dt based on receive timing
            dt = max(0.02, min(0.25, now - t_last))
            t_last = now
            tracker.set_dt(dt)

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
                    pts = parse_tlv1_points(payload)
                    if pts is not None and pts.size > 0:
                        x, y, z, v = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
                        m = (
                            (np.abs(x) < roi_xmax) &
                            (y > roi_ymin) & (y < roi_ymax) &
                            (z > roi_zmin) & (z < roi_zmax)
                        )
                        pts_roi = pts[m]

                offset += (tlv_len - 8)

            # Predict -> cluster -> update
            tracker.predict()

            clusters: List[Cluster] = []
            if pts_roi is not None and pts_roi.shape[0] > 0:
                clusters = cluster_dbscan(pts_roi, eps_m=db_eps, min_samples=db_min_samples)
                clusters = [c for c in clusters if vehicle_like(c)]

            tracker.update(clusters)

            # Update visualization AFTER update so track IDs match current frame
            if viewer is not None:
                conf_now = tracker.confirmed_tracks()
                tracks_for_viz = [(t.tid, float(t.x[0]), float(t.x[1])) for t in conf_now]
                viewer.update(pts_roi, clusters=clusters, tracks=tracks_for_viz)

            # Choose closest confirmed track for printing
            conf = tracker.confirmed_tracks()
            best = None
            best_r = 1e9
            for t in conf:
                x, y = float(t.x[0]), float(t.x[1])
                r = float(np.hypot(x, y))
                if r < best_r:
                    best_r = r
                    best = t

            if now - last_print > 0.5:
                if best is None:
                    npts = 0 if pts_roi is None else int(pts_roi.shape[0])
                    print(f"NO_TARGET | roi_pts={npts} | tracks={len(tracker.tracks)}")
                else:
                    x, y, vx, vy = map(float, best.x)
                    rng = float(np.hypot(x, y))
                    az = float(np.degrees(np.arctan2(x, y)))

                    if rng > 1e-3:
                        los = np.array([x, y], dtype=np.float32) / rng
                        v_r_track = float(los[0] * vx + los[1] * vy)
                    else:
                        v_r_track = 0.0

                    v_r = v_r_track if abs(v_r_track) > 0.15 else float(best.last_vr)

                    print(
                        f"track#{best.tid:02d} | range={rng:6.2f} m | az={az:+6.1f} deg | "
                        f"v_r={v_r:+6.2f} m/s | hits={best.hits:2d} | tracks={len(tracker.tracks)}"
                    )

                last_print = now


# -----------------------------
# CLI
# -----------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viz", action="store_true", help="Show live top-down plot")
    ap.add_argument("--cfg", default="/dev/ttyUSB0", help="CLI/config UART port (115200)")
    ap.add_argument("--data", default="/dev/ttyUSB1", help="Data UART port (921600)")

    ap.add_argument("--fps", type=float, default=10.0, help="Frame rate hint (e.g., 10)")

    # ROI defaults
    ap.add_argument("--xmax", type=float, default=8.0, help="ROI lateral half-width (m)")
    ap.add_argument("--ymin", type=float, default=1.5, help="ROI min range-forward axis (m)")
    ap.add_argument("--ymax", type=float, default=60.0, help="ROI max range-forward axis (m)")
    ap.add_argument("--zmin", type=float, default=-2.5, help="ROI min height (m)")
    ap.add_argument("--zmax", type=float, default=2.5, help="ROI max height (m)")

    # DBSCAN tuned defaults for window/static scenes
    ap.add_argument("--db_eps", type=float, default=1.2, help="DBSCAN eps (m)")
    ap.add_argument("--db_min", type=int, default=3, help="DBSCAN min_samples")

    a = ap.parse_args()

    configure_sensor(a.cfg)
    run_stream(
        data_port=a.data,
        fps_hint=a.fps,
        roi_xmax=a.xmax,
        roi_ymin=a.ymin,
        roi_ymax=a.ymax,
        roi_zmin=a.zmin,
        roi_zmax=a.zmax,
        db_eps=a.db_eps,
        db_min_samples=a.db_min,
        viz=a.viz,
    )


if __name__ == "__main__":
    main()