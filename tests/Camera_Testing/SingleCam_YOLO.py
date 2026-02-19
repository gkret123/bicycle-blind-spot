import cv2
import time
import numpy as np
from collections import deque

from picamera2 import Picamera2

# ---------------- Tunables ----------------
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
FPS_TARGET   = 30

YOLO_MODEL_NAME = "yolov8n.pt"
CONF_THRESH = 0.4
IOU_THRESH  = 0.45
MAX_DETS    = 50
IMG_SZ      = 640          # keep fixed; don't exceed needed size

VEHICLE_CLASS_IDS = {2, 3, 5, 7}  # car, motorcycle, bus, truck (COCO)
CENTER_X_GATE = 0.25             # ±25% of image half-width (more intuitive)
MIN_BBOX_H    = 32

TTC_WARN  = 3.0
TTC_ALERT = 2.0

HIST_LEN   = 10
ALPHA_TTC  = 0.22
CONSEC_ON  = 5
CONSEC_OFF = 8

HFOV_DEG = 90.0

SHOW_DEBUG_SIDE_BY_SIDE = False   # set True if you want it, costs CPU
# ------------------------------------------

def moving_average(deq):
    return sum(deq) / len(deq) if deq else 0.0

def estimate_bearing_deg(cx, w, hfov_deg):
    dx = (cx - (w / 2)) / (w / 2)     # [-1, 1]
    return (hfov_deg / 2.0) * dx

def main():
    from ultralytics import YOLO
    model = YOLO(YOLO_MODEL_NAME)

    # --- Picamera2 setup ---
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "BGR888"},
        controls={"FrameRate": FPS_TARGET},
    )
    picam2.configure(config)
    picam2.start()

    last_time = time.time()
    h_hist = deque(maxlen=HIST_LEN)
    dhdt_hist = deque(maxlen=HIST_LEN)
    ttc_smooth = 99.0
    status = "OK"
    hits_on = 0
    hits_off = 0
    paused = False

    # FPS
    fps_t0 = time.time()
    fps_count = 0
    fps_val = 0.0

    print("Rear-approach (YOLO) on Picamera2. Press 'q' to quit, space to pause/resume.")

    while True:
        if paused:
            time.sleep(0.02)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                paused = False
            elif key == ord('q'):
                break
            continue

        frame = picam2.capture_array("main")  # BGR888 -> OpenCV-native

        H, W = frame.shape[:2]
        cx_mid = W / 2
        x_gate = (W / 2) * CENTER_X_GATE  # pixels

        # YOLO inference
        # NOTE: source=frame is fine; ultralytics will handle numpy arrays.
        results = model.predict(
            source=frame,
            imgsz=IMG_SZ,
            conf=CONF_THRESH,
            iou=IOU_THRESH,
            max_det=MAX_DETS,
            verbose=False
        )

        # Optional debug frame (avoid copy unless needed)
        det_frame = frame.copy() if SHOW_DEBUG_SIDE_BY_SIDE else None

        target = None
        best_score = 1e9

        for r in results:
            if r.boxes is None:
                continue
            boxes = r.boxes
            # Iterate detections
            for b in boxes:
                cls_id = int(b.cls[0].item())
                if cls_id not in VEHICLE_CLASS_IDS:
                    continue

                conf = float(b.conf[0].item())
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                bh = y2 - y1
                bw = x2 - x1
                if bh < MIN_BBOX_H:
                    continue

                cx = x1 + bw / 2
                center_dist = abs(cx - cx_mid)
                if center_dist > x_gate:
                    continue

                # Prefer near-center; slight preference for bigger bbox
                score = center_dist - 0.01 * bh
                if score < best_score:
                    best_score = score
                    target = (x1, y1, x2, y2, conf, cls_id, cx, bh)

                if det_frame is not None:
                    cv2.rectangle(det_frame, (x1, y1), (x2, y2), (80, 80, 80), 1)

        now = time.time()
        dt = max(now - last_time, 1e-6)
        last_time = now

        if target is not None:
            x1, y1, x2, y2, conf, cls_id, cx, bh = target
            h_hist.append(float(bh))

            if len(h_hist) >= 2:
                dhdt_hist.append((h_hist[-1] - h_hist[-2]) / dt)

            dhdt_avg = moving_average(dhdt_hist)
            h_avg = moving_average(h_hist)

            if dhdt_avg > 1e-3:
                ttc = h_avg / dhdt_avg
            else:
                ttc = 99.0

            ttc_smooth = (1 - ALPHA_TTC) * ttc_smooth + ALPHA_TTC * ttc

            # debounce logic
            if ttc_smooth < TTC_ALERT:
                hits_on += 1
                hits_off = max(0, hits_off - 1)
                if hits_on >= CONSEC_ON:
                    status = "ALERT"
            elif ttc_smooth < TTC_WARN:
                hits_on += 1
                hits_off = max(0, hits_off - 1)
                if hits_on >= CONSEC_ON and status != "ALERT":
                    status = "WARN"
            else:
                hits_off += 1
                if hits_off >= CONSEC_OFF:
                    status = "OK"
                    hits_on = 0

            bearing_deg = estimate_bearing_deg(cx, W, HFOV_DEG)

            color = (0, 255, 0) if status == "OK" else ((0, 255, 255) if status == "WARN" else (0, 0, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{cls_id}:{conf:.2f}", (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

            # HUD
            cv2.rectangle(frame, (0, 0), (W, 24), (0, 0, 0), -1)
            hud = f"TTC:{ttc_smooth:4.1f}s  h:{int(h_avg)}px  dh/dt:{dhdt_avg:6.1f}px/s  bear:{bearing_deg:+4.1f}°  {status}"
            cv2.putText(frame, hud, (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

            # console
            print(f"STATUS={status:6s} | h={bh:3.0f}px | dh/dt={dhdt_avg:7.2f}px/s | TTC={ttc_smooth:5.2f}s | bearing={bearing_deg:+5.1f} deg")

        else:
            ttc_smooth = min(99.0, ttc_smooth + 0.2)
            hits_on = max(0, hits_on - 1)
            hits_off += 1
            if hits_off >= CONSEC_OFF:
                status = "OK"
            if int(time.time() * 3) % 15 == 0:
                print("STATUS=NO-TARGET")

        # draw center gate lines
        cv2.line(frame, (int(cx_mid - x_gate), 0), (int(cx_mid - x_gate), H), (80, 80, 80), 1)
        cv2.line(frame, (int(cx_mid + x_gate), 0), (int(cx_mid + x_gate), H), (80, 80, 80), 1)

        # FPS update
        fps_count += 1
        if now - fps_t0 >= 0.5:
            fps_val = fps_count / (now - fps_t0)
            fps_t0 = now
            fps_count = 0

        if SHOW_DEBUG_SIDE_BY_SIDE and det_frame is not None:
            vis = np.hstack([frame, det_frame])
            cv2.putText(vis, f"FPS:{fps_val:4.1f}", (8, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
            cv2.imshow("Rear-approach (YOLO) - Live | Detections", vis)
        else:
            cv2.putText(frame, f"FPS:{fps_val:4.1f}", (8, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
            cv2.imshow("Rear-approach (YOLO)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            paused = not paused

    cv2.destroyAllWindows()
    picam2.stop()

if __name__ == "__main__":
    main()
