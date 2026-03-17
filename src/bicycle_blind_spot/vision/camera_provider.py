"""
Dual-camera vision provider.

Directly ports the proven SingleCam_MultiObjectTracking_YOLO_v2.py approach
to two cameras:
  - Capture BGR888, convert to RGB only for YOLO inference, display BGR as-is
  - Per-track TTC via bbox-height growth rate (EMA smoothed)
  - Status hysteresis: CONSEC_ON / CONSEC_OFF hit counters (same as working script)
  - Best threat = lowest TTC across both cameras, carrying its bearing angle
  - Visualization: gate lines, colored bboxes, TTC + bearing HUD

Note: center-gate is disabled (set to 0.99) because cameras are mounted at ±35°.
A straight-back vehicle appears at the EDGE of each angled camera, so a narrow
center gate would filter it out entirely.
"""

from __future__ import annotations

import time
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
from picamera2 import Picamera2
from ultralytics import YOLO


# ---- Tunables (match SingleCam_MultiObjectTracking_YOLO_v2.py) ----

FRAME_W = 512
FRAME_H = 288
FPS_TARGET = 30

YOLO_MODEL_NAME = "yolov8n.pt"
CONF_THRESH = 0.35
IOU_THRESH = 0.50
MAX_DETS = 20
IMG_SZ = 416

VEHICLE_CLASS_IDS = [2, 3, 5, 7]  # car, motorcycle, bus, truck

MIN_BBOX_H = 14
TRACKER_CFG = "botsort.yaml"
DETECT_EVERY_N_FRAMES = 2
TRACK_STALE_SEC = 0.6

# TTC proxy — identical to working script
HIST_LEN = 12
ALPHA_TTC = 0.12
TTC_WARN = 3.0
TTC_ALERT = 2.0
CONSEC_ON = 7
CONSEC_OFF = 10

# No center-gate for angled cameras (0.99 = accept full frame width)
CENTER_X_GATE = 0.99

# Geometry: per-camera bearing offset from straight-back
HFOV_DEG = 90.0
CAM_L_OFFSET_DEG = -35.0  # left camera mount angle
CAM_R_OFFSET_DEG = +35.0  # right camera mount angle

PRINT_EVERY_SEC = 1.0


@dataclass
class CameraResult:
    """Output of CameraProvider.read()."""
    status: str       # "OK", "WARN", "ALERT", "NO_TARGET"
    ttc_s: float      # best-threat TTC in seconds (99.0 = no threat)
    angle_deg: float  # bearing of best threat (0.0 = straight back)
    n_tracks: int     # total tracks seen across both cameras


def _moving_average(deq: deque) -> float:
    return (sum(deq) / len(deq)) if deq else 0.0


def _status_color(st: str) -> Tuple[int, int, int]:
    if st == "OK":
        return (0, 255, 0)
    if st == "WARN":
        return (0, 255, 255)
    return (0, 0, 255)


def _bearing_deg(cx: float, w: int, hfov_deg: float, mount_offset_deg: float) -> float:
    dx = (cx - w / 2.0) / (w / 2.0)
    return (hfov_deg / 2.0) * dx + mount_offset_deg


class _CamState:
    """Per-camera TTC + hysteresis state, mirroring the working script."""

    def __init__(self):
        self.h_hist = defaultdict(lambda: deque(maxlen=HIST_LEN))
        self.dhdt_hist = defaultdict(lambda: deque(maxlen=HIST_LEN))
        self.ttc_smooth = defaultdict(lambda: 99.0)
        self.status = defaultdict(lambda: "OK")
        self.hits_on = defaultdict(int)
        self.hits_off = defaultdict(int)
        self.last_seen = defaultdict(lambda: 0.0)
        self.last_time = time.time()

    def prune_stale(self, now: float):
        for tid in list(self.last_seen):
            if (now - self.last_seen[tid]) > TRACK_STALE_SEC:
                for d in (self.h_hist, self.dhdt_hist, self.ttc_smooth,
                          self.status, self.hits_on, self.hits_off, self.last_seen):
                    d.pop(tid, None)

    def update(self, tracks: List[dict], now: float) -> Optional[dict]:
        """
        Update TTC and status for all tracks. Returns best (lowest TTC) threat,
        or None if no tracks.
        """
        dt = max(now - self.last_time, 1e-6)
        self.last_time = now
        self.prune_stale(now)

        best = None

        for t in tracks:
            tid = t["tid"]
            self.last_seen[tid] = now

            bh = t["bh"]
            self.h_hist[tid].append(bh)
            if len(self.h_hist[tid]) >= 2:
                self.dhdt_hist[tid].append(
                    (self.h_hist[tid][-1] - self.h_hist[tid][-2]) / dt
                )

            dhdt_avg = _moving_average(self.dhdt_hist[tid])
            h_avg = _moving_average(self.h_hist[tid])

            ttc = (h_avg / dhdt_avg) if dhdt_avg > 0.5 else 99.0
            self.ttc_smooth[tid] = (1.0 - ALPHA_TTC) * self.ttc_smooth[tid] + ALPHA_TTC * ttc

            # Hysteresis state machine — identical to working script
            s = self.ttc_smooth[tid]
            if s < TTC_ALERT:
                self.hits_on[tid] += 1
                self.hits_off[tid] = max(0, self.hits_off[tid] - 1)
                if self.hits_on[tid] >= CONSEC_ON:
                    self.status[tid] = "ALERT"
            elif s < TTC_WARN:
                self.hits_on[tid] += 1
                self.hits_off[tid] = max(0, self.hits_off[tid] - 1)
                if self.hits_on[tid] >= CONSEC_ON and self.status[tid] != "ALERT":
                    self.status[tid] = "WARN"
            else:
                self.hits_off[tid] += 1
                if self.hits_off[tid] >= CONSEC_OFF:
                    self.status[tid] = "OK"
                    self.hits_on[tid] = 0

            if best is None or self.ttc_smooth[tid] < best["ttc"]:
                best = {
                    "tid": tid,
                    "ttc": self.ttc_smooth[tid],
                    "status": self.status[tid],
                    "cx": t["cx"],
                }

        return best


class CameraProvider:
    """
    Dual Picamera2 + YOLO provider that directly ports the proven
    SingleCam_MultiObjectTracking_YOLO_v2.py detection loop.
    """

    def __init__(
        self,
        cam_left_index: int = 0,
        cam_right_index: int = 1,
        show: bool = False,
    ):
        """
        Args:
            cam_left_index: Pi camera index for left camera.
            cam_right_index: Pi camera index for right camera.
            show: If True, display cv2.imshow windows.
        """
        self.show = show

        available = Picamera2.global_camera_info()
        if len(available) < 2:
            print(f"[camera] Warning: only {len(available)} camera(s) found — "
                  "using camera 0 for both channels.")
            cam_left_index = 0
            cam_right_index = 0

        self.cam_left = Picamera2(camera_num=cam_left_index)
        self._dual = cam_left_index != cam_right_index
        self.cam_right = Picamera2(camera_num=cam_right_index) if self._dual else self.cam_left

        vid_cfg = dict(
            main={"size": (FRAME_W, FRAME_H), "format": "BGR888"},
            controls={"FrameRate": FPS_TARGET},
        )
        for cam in ([self.cam_left, self.cam_right] if self._dual else [self.cam_left]):
            cam.configure(cam.create_video_configuration(**vid_cfg))
            cam.start()

        self.model = YOLO(YOLO_MODEL_NAME)

        self._state_l = _CamState()
        self._state_r = _CamState()

        self._frame_idx = 0
        self._cached_l = None
        self._cached_r = None

        self._fps_t0 = time.time()
        self._fps_count = 0
        self._fps_val = 0.0
        self._last_print = time.time()

    def stop(self):
        for cam in ([self.cam_left, self.cam_right] if self._dual else [self.cam_left]):
            try:
                cam.stop()
            except Exception:
                pass

    def _parse_tracks(self, results, H: int, W: int) -> List[dict]:
        """Extract valid detections from YOLO results — identical filter to working script."""
        tracks = []
        cx_mid = W / 2.0
        x_gate = cx_mid * CENTER_X_GATE

        for r in results:
            if r.boxes is None:
                continue
            for b in r.boxes:
                if b.id is None:
                    continue
                tid = int(b.id[0].item())
                cls_id = int(b.cls[0].item())
                conf = float(b.conf[0].item())
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())

                bh = y2 - y1
                bw = x2 - x1
                if bh < MIN_BBOX_H:
                    continue

                cx = x1 + bw / 2.0
                if abs(cx - cx_mid) > x_gate:
                    continue

                tracks.append({
                    "tid": tid, "cls": cls_id, "conf": conf,
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                    "cx": cx, "bh": float(bh),
                })
        return tracks

    def _render(
        self,
        frame_bgr,
        tracks: List[dict],
        state: _CamState,
        mount_offset: float,
        best: Optional[dict],
        label: str,
    ):
        H, W = frame_bgr.shape[:2]
        cx_mid = W / 2.0
        x_gate = cx_mid * CENTER_X_GATE

        # Gate lines
        cv2.line(frame_bgr, (int(cx_mid - x_gate), 0), (int(cx_mid - x_gate), H), (80, 80, 80), 1)
        cv2.line(frame_bgr, (int(cx_mid + x_gate), 0), (int(cx_mid + x_gate), H), (80, 80, 80), 1)

        for t in tracks:
            tid = t["tid"]
            st = state.status[tid]
            color = _status_color(st)
            bearing = _bearing_deg(t["cx"], W, HFOV_DEG, mount_offset)

            cv2.rectangle(frame_bgr, (t["x1"], t["y1"]), (t["x2"], t["y2"]), color, 2)
            cv2.putText(frame_bgr, f"ID{tid} {t['conf']:.2f}",
                        (t["x1"], max(0, t["y1"] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            cv2.putText(frame_bgr, f"TTC {state.ttc_smooth[tid]:.1f}s {bearing:+.1f}°",
                        (t["x1"], min(H - 5, t["y2"] + 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        # HUD bar
        global_st = best["status"] if best else "NO_TARGET"
        color = _status_color(global_st) if best else (200, 200, 200)
        cv2.rectangle(frame_bgr, (0, 0), (W, 28), (0, 0, 0), -1)
        threat_txt = ""
        if best:
            bearing = _bearing_deg(best["cx"], W, HFOV_DEG, mount_offset)
            threat_txt = f" | ID{best['tid']} TTC={best['ttc']:.1f}s {bearing:+.1f}°"
        cv2.putText(frame_bgr, f"{label} {global_st}{threat_txt}",
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        cv2.putText(frame_bgr, f"FPS:{self._fps_val:.1f}",
                    (8, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

    def read(self) -> CameraResult:
        """
        Capture, detect, update TTC/status, optionally render, return CameraResult.
        """
        now = time.time()

        frame_l = self.cam_left.capture_array("main")
        frame_r = self.cam_right.capture_array("main") if self._dual else frame_l.copy()

        H, W = frame_l.shape[:2]

        # YOLO expects RGB — convert only for inference, display BGR as-is
        rgb_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2RGB)
        rgb_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2RGB)

        # Detection every N frames, cache between runs
        self._frame_idx += 1
        run_det = (self._frame_idx % DETECT_EVERY_N_FRAMES) == 0 or self._cached_l is None

        if run_det:
            self._cached_l = self.model.track(
                source=rgb_l, imgsz=IMG_SZ, conf=CONF_THRESH, iou=IOU_THRESH,
                max_det=MAX_DETS, classes=VEHICLE_CLASS_IDS,
                verbose=False, persist=True, tracker=TRACKER_CFG,
            )
            self._cached_r = self.model.track(
                source=rgb_r, imgsz=IMG_SZ, conf=CONF_THRESH, iou=IOU_THRESH,
                max_det=MAX_DETS, classes=VEHICLE_CLASS_IDS,
                verbose=False, persist=True, tracker=TRACKER_CFG,
            )

        tracks_l = self._parse_tracks(self._cached_l, H, W)
        tracks_r = self._parse_tracks(self._cached_r, H, W)

        best_l = self._state_l.update(tracks_l, now)
        best_r = self._state_r.update(tracks_r, now)

        # Pick best threat across both cameras
        if best_l is not None and best_r is not None:
            best = best_l if best_l["ttc"] <= best_r["ttc"] else best_r
            best_cam = "left" if best is best_l else "right"
        elif best_l is not None:
            best = best_l
            best_cam = "left"
        elif best_r is not None:
            best = best_r
            best_cam = "right"
        else:
            best = None
            best_cam = None

        # FPS
        self._fps_count += 1
        if now - self._fps_t0 >= 0.5:
            self._fps_val = self._fps_count / (now - self._fps_t0)
            self._fps_t0 = now
            self._fps_count = 0

        # Visualization
        if self.show:
            self._render(frame_l, tracks_l, self._state_l, CAM_L_OFFSET_DEG, best_l, "LEFT")
            self._render(frame_r, tracks_r, self._state_r, CAM_R_OFFSET_DEG, best_r, "RIGHT")
            cv2.imshow("Left Camera", frame_l)
            cv2.imshow("Right Camera", frame_r)
            cv2.waitKey(1)

        # Throttled console print
        if now - self._last_print >= PRINT_EVERY_SEC:
            self._last_print = now
            n = len(tracks_l) + len(tracks_r)
            if best:
                mount = CAM_L_OFFSET_DEG if best_cam == "left" else CAM_R_OFFSET_DEG
                bearing = _bearing_deg(best["cx"], W, HFOV_DEG, mount)
                print(f"[{time.strftime('%H:%M:%S')}] FPS~{self._fps_val:.1f} "
                      f"tracks={n} status={best['status']} "
                      f"TTC={best['ttc']:.1f}s angle={bearing:+.1f}°")
            else:
                print(f"[{time.strftime('%H:%M:%S')}] FPS~{self._fps_val:.1f} "
                      f"tracks=0 status=NO_TARGET")

        # Build result
        if best is None:
            return CameraResult(status="NO_TARGET", ttc_s=99.0, angle_deg=0.0, n_tracks=0)

        mount = CAM_L_OFFSET_DEG if best_cam == "left" else CAM_R_OFFSET_DEG
        bearing = _bearing_deg(best["cx"], W, HFOV_DEG, mount)

        return CameraResult(
            status=best["status"],
            ttc_s=best["ttc"],
            angle_deg=bearing,
            n_tracks=len(tracks_l) + len(tracks_r),
        )
