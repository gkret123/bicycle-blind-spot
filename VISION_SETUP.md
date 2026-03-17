# Dual-Camera Vision TTR System

## Overview
This system uses two rear-facing Raspberry Pi cameras with YOLO object detection to:
1. Detect approaching vehicles
2. Estimate their bearing angle (relative to straight back)
3. Compute time-to-react (TTR) based on approach speed and distance
4. Split the warning signal between left/right haptic devices based on which side the threat is approaching from

## Hardware Setup

### Camera Mounting (Recommended: V-Configuration)
Mount the two Pi cameras in a "V" shape with a ~20cm baseline:
- **Left camera**: Angled ~35° to the left of straight-back
- **Right camera**: Angled ~35° to the right of straight-back
- **Result**: ~140° combined rear coverage with stereo triangulation capability

This configuration:
- Provides natural left/right assignment (left cam angle → left side vibration)
- Allows range triangulation if the same vehicle is seen in both cameras
- Gives reasonable overlap in the center for straight-back approaches

### Alternative Configurations
- **Both straight back**: Simpler mounting, still works (uses bbox-height-only range estimation)
- **Different angles**: Adjust `CAM_L_OFFSET_DEG` and `CAM_R_OFFSET_DEG` in `camera_provider.py`

## Calibration

### Focal Length (FOCAL_PX)
For accurate range estimation via stereo triangulation, you need to calibrate the focal length.

Quick test:
```bash
python3 -c "
from picamera2 import Picamera2
import cv2
cam = Picamera2()
config = cam.create_video_configuration(main={'size': (512, 288), 'format': 'BGR888'})
cam.configure(config)
cam.start()
frame = cam.capture_array()
print(f'Frame shape: {frame.shape}')  # Should be (288, 512, 3)
cam.stop()
"
```

Then use a known-size object (e.g., a car at a measured distance) to back-solve:
```
focal_px = (bbox_width * distance_m) / actual_object_width
```

Default is ~320 pixels at 512px width; adjust in `camera_provider.py` if needed.

### Stereo Baseline (STEREO_BASELINE_M)
Measure the horizontal distance between the two camera centers in meters. Default is 0.20m (20cm).

## Running the Vision-Based TTR System

### Basic (with visualization)
```bash
cd /home/gkret/Documents/bicycle-blind-spot
source .venv/bin/activate
python scripts/ble_send_ttr_vision.py --headless false --visualize true
```

**Output:**
- Two OpenCV windows showing left/right camera feeds with bounding boxes
- Console prints TTR, angle, and threat status every ~0.5s

**Controls:**
- `q` or Ctrl+C: quit
- Left/right/straight approach: see differences in left/right vibration intensity

### Production (no GUI, autostart-ready)
```bash
python scripts/ble_send_ttr_vision.py --headless true
```

Or (if installed for autostart):
```bash
sudo systemctl start bicycle-blind-spot.service
```

### Tuning Parameters

| Parameter | Default | Purpose | Notes |
|-----------|---------|---------|-------|
| `--hz` | 20 | BLE send rate (Hz) | Higher = more responsive, more BLE traffic |
| `--ttr-warn` | 3.0 | TTR threshold for "warn" state (seconds) | Adjust based on cyclist reaction time |
| `--ttr-alert` | 1.8 | TTR threshold for "alert" state (seconds) | Closer = more urgent |
| `--angle-boost` | 0.7 | How aggressively to split L/R (0.0–1.0) | 0.0 = symmetric, 1.0 = max L/R differentiation |
| `--cam-left` | 0 | Pi camera index for left camera | Check with `vcgencmd get_camera` |
| `--cam-right` | 1 | Pi camera index for right camera | Same as above |

### Example: High Sensitivity, Aggressive L/R Split
```bash
python scripts/ble_send_ttr_vision.py \
  --headless true \
  --ttr-alert 2.0 \
  --angle-boost 1.0 \
  --hz 30
```

## Output

### Console Output Example
```
[12:34:56] TTR base=0.850 L=0.920 R=0.780 angle=-25.3° status=OK hz=20.1 L:ok R:ok
[12:34:57] TTR base=0.720 L=0.680 R=0.880 angle=-15.0° status=WARN hz=20.0 L:ok R:ok
[12:34:58] TTR base=0.380 L=0.200 R=0.950 angle=-45.0° status=ALERT hz=20.1 L:ok R:ok
```

**Fields:**
- `TTR base`: Overall urgency (1.0 = calm, 0.0 = urgent)
- `L`, `R`: Per-device TTR after angle split (left device gets more urgent signal if angle < 0)
- `angle`: Vehicle bearing in degrees (negative = left, positive = right)
- `status`: Threat level (OK / WARN / ALERT / NO_TARGET)
- `hz`: Actual send frequency

### Video Overlay (when `--visualize true`)
- **Green boxes**: Detected vehicles with bounding boxes
- **Bearing angle**: Labeled below each detection (in degrees)
- **TTC (Time-to-Collision)**: Computed from bounding-box growth rate
- **Gate lines**: ±40% of frame width (tracks outside this are ignored)
- **Mode**: FPS, detection interval

## Angle-to-L/R Split Logic

The system computes a **split_factor** from the vehicle bearing angle:
```python
angle_clamped = clamp(angle_deg, -70°, +70°)
split_factor = angle_clamped / 70.0
```

Then applies to each side:
```python
left_ttr  = base_ttr + (1 - base_ttr) * max(0, -split_factor) * angle_boost
right_ttr = base_ttr + (1 - base_ttr) * max(0, +split_factor) * angle_boost
```

**Examples (with `angle_boost=0.7`):**

| Scenario | Angle | split_factor | left_ttr | right_ttr | Effect |
|----------|-------|--------------|----------|-----------|--------|
| Straight back | 0° | 0.0 | base | base | Both sides equal |
| Left approach | -45° | -0.64 | base - 0.45×(1-base) | base + 0.00 | Left urgent, right calm |
| Right approach | +45° | +0.64 | base + 0.00 | base - 0.45×(1-base) | Right urgent, left calm |
| Far left | -70° | -1.0 | base - 0.70×(1-base) | base + 0.00 | Maximal left differentiation |

## Troubleshooting

### "BLE device not found"
- Check device names: `bluetoothctl scan on` (Ctrl+C to stop)
- Confirm ESP32 units are powered and advertising
- Verify `--left-device` and `--right-device` match advertised names

### No detections / all "NO_TARGET"
1. Run with `--visualize true` to see camera feeds
2. Walk toward the cameras from various angles
3. If no boxes appear:
   - Check YOLO model loaded: `python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"`
   - Verify camera indices: `vcgencmd get_camera`
   - Ensure good lighting

### Angle always 0° or wrong bearing
- Check `HFOV_DEG` matches your lenses (default assumes ~90° FOV)
- Verify camera mounting offsets (`CAM_L_OFFSET_DEG`, `CAM_R_OFFSET_DEG`)
- Check frame width is indeed 512px (printed on startup)

### Low FPS (<15 Hz)
- Reduce detection frequency: `DETECT_EVERY_N_FRAMES=3` or higher
- Reduce frame size: `FRAME_W=384, FRAME_H=216`
- Reduce YOLO model size: use `yolov8n.pt` (nano, default) instead of larger models
- Close other processes

### L/R split not changing with angle
- Increase `--angle-boost` toward 1.0
- Verify vehicle is actually detected (check `--visualize true`)
- Check vehicle is within the horizontal gate (±40% of frame width)

## Testing Procedure

1. **Static test** (no vehicle):
   ```bash
   python scripts/ble_send_ttr_vision.py --headless false --visualize true
   ```
   Both devices should remain calm (TTR=1.0), no threat status.

2. **Linear approach** (straight toward camera):
   ```
   [walk/ride straight toward camera]
   ```
   Both devices should vibrate equally; angle stays ~0°; TTR decreases.

3. **Left approach**:
   ```
   [walk/ride from the left at an angle toward camera]
   ```
   Left device vibrates more (lower TTR); right device eased up; angle < 0°.

4. **Right approach**:
   ```
   [walk/ride from the right at an angle toward camera]
   ```
   Right device vibrates more; left device eased up; angle > 0°.

5. **Lateral pass** (sideway motion):
   ```
   [walk/ride laterally across rear view]
   ```
   Should see angle swing from -45° to +45°; L/R split follows.

6. **Test with both BLE devices connected**:
   Check both left and right devices vibrate with expected intensities.

## Files Modified / Created

- **Created**: `src/bicycle_blind_spot/vision/camera_provider.py` — dual-camera YOLO tracking
- **Created**: `src/bicycle_blind_spot/vision/ttr_source_vision.py` — threat engine + angle split
- **Created**: `scripts/ble_send_ttr_vision.py` — main entry point
- **Created**: `src/bicycle_blind_spot/vision/__init__.py` — module exports
- **Updated**: `src/Review/threat_algo.py` — unchanged, reused as-is

## Future Enhancements

1. **Stereo range refinement**: Currently falls back to bbox-height proxy; implement proper disparity matching for more accurate range when both cameras see the same vehicle.

2. **Multi-vehicle tracking**: Currently picks the single most-dangerous threat; could expand to track multiple simultaneous threats.

3. **Deep learning focal loss**: Fine-tune YOLO on bicycle/car rear detection for better accuracy.

4. **Sensor fusion**: Combine with actual radar (TI mmWave) for redundancy and improved range/speed estimates.

5. **Adaptive thresholds**: Adjust TTR warn/alert thresholds based on cyclist speed (if available via Bluetooth from bike computer).

## References

- YOLO tracking: https://docs.ultralytics.com/modes/track/
- Picamera2 API: https://github.com/raspberrypi/picamera2
- Threat algorithm: `src/Review/threat_algo.py`
