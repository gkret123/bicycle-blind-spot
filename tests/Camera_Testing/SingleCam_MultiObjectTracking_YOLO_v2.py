#!/usr/bin/env python3
"""
Pi 5 tuned: Picamera2 + YOLOv8 + tracker (BoT-SORT default)

Fixes:
- Color correctness: capture BGR888 (OpenCV-native), display as-is
- Feed YOLO RGB explicitly (BGR->RGB conversion only for inference)
- Boxes disappear quickly when vehicle leaves frame (TRACK_STALE_SEC)
- Multi vs closest-only toggle ('m')
- Pause ('space'), quit ('q'), toggle visualization ('v')

Optional channel sanity test:
- Draw a "RED" patch; if it shows blue, uncomment the one-line channel swap.
"""

import time
import cv2
from collections import deque, defaultdict

from picamera2 import Picamera2


# ---------------- Tunables ----------------
FRAME_WIDTH  = 512
FRAME_HEIGHT = 288
FPS_TARGET   = 30

YOLO_MODEL_NAME = "yolov8n.pt"
CONF_THRESH = 0.35
IOU_THRESH  = 0.50
MAX_DETS    = 20
IMG_SZ      = 416

# COCO vehicle classes: car=2, motorcycle=3, bus=5, truck=7
VEHICLE_CLASS_IDS = [2, 3, 5, 7]

CENTER_X_GATE = 0.40
MIN_BBOX_H    = 14

TRACKER_CFG = "botsort.yaml"      # try "bytetrack.yaml" too

MULTI_MODE_DEFAULT = True
VISUALIZE_DEFAULT  = True

# Speed: run YOLO every N frames
DETECT_EVERY_N_FRAMES = 2         # 1=best quality, 2/3=faster

# Remove tracks fast when not seen (boxes disappear quickly when leaving frame)
TRACK_STALE_SEC = 0.6

# TTC proxy (camera-only; noisy at distance; kept for now)
HIST_LEN   = 12
ALPHA_TTC  = 0.12
TTC_WARN   = 3.0
TTC_ALERT  = 2.0
CONSEC_ON  = 7
CONSEC_OFF = 10

HFOV_DEG = 90.0

PRINT_EVERY_SEC = 1.0

# Debug color patch: draws a red square; if it appears blue, swap channels once.
DRAW_COLOR_TEST_PATCH = False
# ------------------------------------------


def moving_average(deq: deque) -> float:
    return (sum(deq) / len(deq)) if deq else 0.0


def estimate_bearing_deg(cx: float, w: int, hfov_deg: float) -> float:
    dx = (cx - (w / 2.0)) / (w / 2.0)
    return (hfov_deg / 2.0) * dx


def status_color(st: str):
    # OpenCV BGR
    if st == "OK":
        return (0, 255, 0)
    if st == "WARN":
        return (0, 255, 255)
    return (0, 0, 255)


def pick_closest_track(tracks):
    # closest ~ biggest bbox height
    return max(tracks, key=lambda t: t["bh"])


def main():
    from ultralytics import YOLO
    model = YOLO(YOLO_MODEL_NAME)

    # --- Picamera2 setup ---
    # Capture in BGR888 (OpenCV-native). DO NOT convert before imshow.
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "BGR888"},
        controls={"FrameRate": FPS_TARGET},
    )
    picam2.configure(config)
    picam2.start()

    # Per-track state
    h_hist = defaultdict(lambda: deque(maxlen=HIST_LEN))
    dhdt_hist = defaultdict(lambda: deque(maxlen=HIST_LEN))
    ttc_smooth = defaultdict(lambda: 99.0)
    status = defaultdict(lambda: "OK")
    hits_on = defaultdict(int)
    hits_off = defaultdict(int)
    last_seen = defaultdict(lambda: 0.0)

    paused = False
    multi_mode = MULTI_MODE_DEFAULT
    visualize = VISUALIZE_DEFAULT

    last_time = time.time()

    # FPS calc
    fps_t0 = time.time()
    fps_count = 0
    fps_val = 0.0

    # Detection decimation
    frame_idx = 0
    cached_results = None

    # Throttled printing
    last_print = time.time()

    print("Rear-approach (YOLO+Track) on Picamera2 (Pi 5 tuned)")
    print("Keys: q quit | space pause | m MULTI/CLOSEST | v visualize on/off")

    try:
        while True:
            if paused:
                time.sleep(0.02)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):
                    paused = False
                elif key == ord('q'):
                    break
                elif key == ord('m'):
                    multi_mode = not multi_mode
                    print(f"MODE={'MULTI' if multi_mode else 'CLOSEST'}")
                elif key == ord('v'):
                    visualize = not visualize
                    if not visualize:
                        cv2.destroyAllWindows()
                    print(f"VISUALIZE={'ON' if visualize else 'OFF'}")
                continue

            frame_bgr = picam2.capture_array("main")  # OpenCV-native

            # If your red test patch shows BLUE, uncomment this:
            # frame_bgr = frame_bgr[:, :, ::-1]

            H, W = frame_bgr.shape[:2]
            cx_mid = W / 2.0
            x_gate = (W / 2.0) * CENTER_X_GATE

            # YOLO expects RGB; convert ONLY for inference
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            # Run detector/tracker every N frames
            frame_idx += 1
            run_det = (frame_idx % DETECT_EVERY_N_FRAMES) == 0 or cached_results is None
            if run_det:
                cached_results = model.track(
                    source=frame_rgb,
                    imgsz=IMG_SZ,
                    conf=CONF_THRESH,
                    iou=IOU_THRESH,
                    max_det=MAX_DETS,
                    classes=VEHICLE_CLASS_IDS,
                    verbose=False,
                    persist=True,
                    tracker=TRACKER_CFG
                )
            results = cached_results

            now = time.time()
            dt = max(now - last_time, 1e-6)
            last_time = now

            # Cleanup stale tracks quickly
            for tid, t_seen in list(last_seen.items()):
                if (now - t_seen) > TRACK_STALE_SEC:
                    last_seen.pop(tid, None)
                    h_hist.pop(tid, None)
                    dhdt_hist.pop(tid, None)
                    ttc_smooth.pop(tid, None)
                    status.pop(tid, None)
                    hits_on.pop(tid, None)
                    hits_off.pop(tid, None)

            # Parse detections/tracks
            tracks = []
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
                        "tid": tid,
                        "cls": cls_id,
                        "conf": conf,
                        "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                        "cx": cx,
                        "bh": float(bh),
                    })

            # Closest-only mode
            if tracks and not multi_mode:
                tracks = [pick_closest_track(tracks)]

            # Update TTC/status (per-track)
            global_state = "OK"
            best_threat = None

            for t in tracks:
                tid = t["tid"]
                last_seen[tid] = now

                bh = t["bh"]
                h_hist[tid].append(bh)
                if len(h_hist[tid]) >= 2:
                    dhdt_hist[tid].append((h_hist[tid][-1] - h_hist[tid][-2]) / dt)

                dhdt_avg = moving_average(dhdt_hist[tid])
                h_avg = moving_average(h_hist[tid])

                if dhdt_avg > 0.5:
                    ttc = h_avg / dhdt_avg
                else:
                    ttc = 99.0

                ttc_smooth[tid] = (1.0 - ALPHA_TTC) * ttc_smooth[tid] + ALPHA_TTC * ttc

                if ttc_smooth[tid] < TTC_ALERT:
                    hits_on[tid] += 1
                    hits_off[tid] = max(0, hits_off[tid] - 1)
                    if hits_on[tid] >= CONSEC_ON:
                        status[tid] = "ALERT"
                elif ttc_smooth[tid] < TTC_WARN:
                    hits_on[tid] += 1
                    hits_off[tid] = max(0, hits_off[tid] - 1)
                    if hits_on[tid] >= CONSEC_ON and status[tid] != "ALERT":
                        status[tid] = "WARN"
                else:
                    hits_off[tid] += 1
                    if hits_off[tid] >= CONSEC_OFF:
                        status[tid] = "OK"
                        hits_on[tid] = 0

                if status[tid] == "ALERT":
                    global_state = "ALERT"
                elif status[tid] == "WARN" and global_state != "ALERT":
                    global_state = "WARN"

                if best_threat is None or ttc_smooth[tid] < best_threat["ttc"]:
                    best_threat = {"tid": tid, "ttc": ttc_smooth[tid], "status": status[tid]}

            # FPS
            fps_count += 1
            if now - fps_t0 >= 0.5:
                fps_val = fps_count / (now - fps_t0)
                fps_t0 = now
                fps_count = 0

            # Visualization (NO conversions needed; frame is already BGR)
            if visualize:
                if DRAW_COLOR_TEST_PATCH:
                    # Should look RED (BGR=(0,0,255))
                    cv2.rectangle(frame_bgr, (10, 40), (60, 90), (0, 0, 255), -1)
                    cv2.putText(frame_bgr, "RED", (10, 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # gate lines
                cv2.line(frame_bgr, (int(cx_mid - x_gate), 0), (int(cx_mid - x_gate), H), (80, 80, 80), 1)
                cv2.line(frame_bgr, (int(cx_mid + x_gate), 0), (int(cx_mid + x_gate), H), (80, 80, 80), 1)

                for t in tracks:
                    tid = t["tid"]
                    st = status[tid]
                    color = status_color(st)

                    cv2.rectangle(frame_bgr, (t["x1"], t["y1"]), (t["x2"], t["y2"]), color, 2)
                    bearing = estimate_bearing_deg(t["cx"], W, HFOV_DEG)

                    cv2.putText(frame_bgr, f"ID{tid} {t['cls']} {t['conf']:.2f}",
                                (t["x1"], max(0, t["y1"] - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                    cv2.putText(frame_bgr, f"TTC {ttc_smooth[tid]:.1f}s {bearing:+.1f}deg",
                                (t["x1"], min(H - 5, t["y2"] + 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

                # HUD
                cv2.rectangle(frame_bgr, (0, 0), (W, 28), (0, 0, 0), -1)
                mode_txt = "MULTI" if multi_mode else "CLOSEST"
                threat_txt = ""
                if best_threat:
                    threat_txt = f" | Threat ID{best_threat['tid']} {best_threat['status']} TTC {best_threat['ttc']:.1f}s"

                cv2.putText(frame_bgr,
                            f"MODE:{mode_txt} GLOBAL:{global_state}{threat_txt}",
                            (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            status_color(global_state), 1, cv2.LINE_AA)

                cv2.putText(frame_bgr, f"FPS:{fps_val:4.1f} DET_EVERY:{DETECT_EVERY_N_FRAMES} IMG_SZ:{IMG_SZ}",
                            (8, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (240, 240, 240), 1, cv2.LINE_AA)

                cv2.imshow("Rear-approach (Pi5 tuned)", frame_bgr)

            # Throttled printing
            if (now - last_print) >= PRINT_EVERY_SEC:
                last_print = now
                if best_threat:
                    print(f"[{time.strftime('%H:%M:%S')}] FPS~{fps_val:4.1f} tracks={len(tracks)} "
                          f"global={global_state} threat=ID{best_threat['tid']} {best_threat['status']} TTC={best_threat['ttc']:.1f}s")
                else:
                    print(f"[{time.strftime('%H:%M:%S')}] FPS~{fps_val:4.1f} tracks=0 global={global_state}")

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                paused = not paused
            elif key == ord('m'):
                multi_mode = not multi_mode
                print(f"MODE={'MULTI' if multi_mode else 'CLOSEST'}")
            elif key == ord('v'):
                visualize = not visualize
                if not visualize:
                    cv2.destroyAllWindows()
                print(f"VISUALIZE={'ON' if visualize else 'OFF'}")

    finally:
        cv2.destroyAllWindows()
        try:
            picam2.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()