# Quick Start: Dual-Camera Vision TTR

## TL;DR

You now have a working computer vision system that:
- Uses 2 rear Pi cameras to detect approaching vehicles
- Measures their angle (left/right)
- Splits the haptic warning between left/right devices based on which side they're on

## Files Created
```
src/bicycle_blind_spot/vision/
├── camera_provider.py        # YOLO tracking + angle/range computation
├── ttr_source_vision.py      # Threat assessment + L/R split logic
└── __init__.py               # Module exports

scripts/
└── ble_send_ttr_vision.py    # Main entry point (new)

VISION_SETUP.md               # Comprehensive guide
QUICKSTART_VISION.md          # This file
```

## Camera Mounting
**Recommended: V-configuration**
- Left camera: -35° from straight-back
- Right camera: +35° from straight-back
- Baseline: 20cm apart
- Result: ~140° rear coverage with left/right differentiation

*(You can use other configs; see VISION_SETUP.md)*

## First Run (with visualization)

```bash
cd /home/gkret/Documents/bicycle-blind-spot
source .venv/bin/activate

python scripts/ble_send_ttr_vision.py --headless false --visualize true
```

**You should see:**
- Two OpenCV windows (left and right camera feeds)
- Bounding boxes around detected vehicles
- Bearing angle (in degrees) for each detection
- Console output showing TTR, angle, and connection status
- No vehicle present: `status=NO_TARGET`, `TTR=1.0` (calm)

**Keys:**
- `q` or Ctrl+C: quit
- Walk toward the cameras from different angles to see the system respond

## Production Mode (no GUI)

```bash
python scripts/ble_send_ttr_vision.py --headless true
```

Output:
```
[12:34:56] TTR base=0.850 L=0.920 R=0.780 angle=-25.3° status=OK hz=20.1 L:ok R:ok
```

**Fields:**
- `TTR base`: Overall urgency (1.0=calm, 0.0=danger)
- `L`, `R`: Left/right device TTR (after angle split)
- `angle`: Vehicle bearing (-ve=left, +ve=right)
- `status`: Threat level (OK / WARN / ALERT / NO_TARGET)

## How the L/R Split Works

**Simple rule:** The closer side gets more haptic feedback.

Examples:
- **Straight approach** (angle ≈ 0°) → Both sides equal
- **Left approach** (angle ≈ -45°) → Left device vibrates more
- **Right approach** (angle ≈ +45°) → Right device vibrates more
- **Far left** (angle ≈ -70°) → Left device maximal intensity

Control with `--angle-boost`:
- `0.0` = L/R always equal (only base TTR matters)
- `0.7` = Moderate split (default, good balance)
- `1.0` = Aggressive split (very directional)

```bash
# Max directional feedback:
python scripts/ble_send_ttr_vision.py --headless true --angle-boost 1.0

# Symmetric warning (ignore angle):
python scripts/ble_send_ttr_vision.py --headless true --angle-boost 0.0
```

## Test Scenarios

1. **No vehicle present**
   ```bash
   [run script with no one nearby]
   Expected: status=NO_TARGET, TTR≈1.0, both devices calm
   ```

2. **Walk straight toward cameras**
   ```bash
   Expected: angle≈0°, both devices vibrate equally, TTR decreases
   ```

3. **Walk from left toward cameras**
   ```bash
   Expected: angle<0°, left device more intense, angle changes as you move
   ```

4. **Walk from right toward cameras**
   ```bash
   Expected: angle>0°, right device more intense
   ```

5. **Walk sideways across rear**
   ```bash
   Expected: angle swings from -45° to +45°, L/R split follows smoothly
   ```

## Tuning Parameters

### Most Important
- `--angle-boost` (0.0–1.0) — how aggressive the L/R split is
- `--ttr-alert` (seconds) — when to trigger "ALERT" state
- `--ttr-warn` (seconds) — when to trigger "WARN" state

### Example Profiles

**Conservative (high threshold):**
```bash
python scripts/ble_send_ttr_vision.py \
  --headless true \
  --ttr-warn 4.0 \
  --ttr-alert 2.5 \
  --angle-boost 0.5
```

**Aggressive (low threshold):**
```bash
python scripts/ble_send_ttr_vision.py \
  --headless true \
  --ttr-warn 2.0 \
  --ttr-alert 1.0 \
  --angle-boost 1.0 \
  --hz 30
```

**See all options:**
```bash
python scripts/ble_send_ttr_vision.py --help
```

## Troubleshooting

### "BLE device not found"
- Check ESP32 units are powered and advertising
- Verify device names: `--left-device "BBSpot-XIAO-L" --right-device "BBSpot-XIAO-R"`

### No vehicle detections (all "NO_TARGET")
- Run with `--visualize true` to see camera feeds
- Check lighting
- Walk closer to cameras
- Verify YOLO model is loaded: `python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"`

### Wrong angle (always 0° or inverted)
- Verify camera indices: `vcgencmd get_camera`
- Check `CAM_L_OFFSET_DEG` and `CAM_R_OFFSET_DEG` in camera_provider.py match your mounting
- Verify HFOV_DEG (field of view) is correct for your lenses

### Low FPS
- Reduce detection frequency (tunable in camera_provider.py)
- Use smaller frame size
- Close other processes

See **VISION_SETUP.md** for detailed troubleshooting.

## Next Steps

1. **Mount the cameras** in V-configuration
2. **Run with visualization** and test all scenarios
3. **Tune `--angle-boost`** to your preference
4. **Adjust threat thresholds** (`--ttr-warn`, `--ttr-alert`)
5. **Run in production** with `--headless true`
6. **(Optional) Set up autostart** via systemd

## Autostart Setup

Update `/etc/systemd/system/bicycle-blind-spot.service`:
```ini
[Service]
ExecStart=/home/pi/bicycle-blind-spot/scripts/pi/run_bbs_vision.sh
```

Create `scripts/pi/run_bbs_vision.sh`:
```bash
#!/bin/bash
cd /home/pi/bicycle-blind-spot
source .venv/bin/activate
python scripts/ble_send_ttr_vision.py --headless true
```

Then:
```bash
sudo systemctl daemon-reload
sudo systemctl restart bicycle-blind-spot.service
```

## Fallback

If you need the old ramp-based system:
```bash
python scripts/ble_send_ttr.py  # Still works, unchanged
```

The vision system is independent, so you can switch freely.

## Key Metrics to Monitor

**In console output, watch these:**

| Metric | Meaning | Action if... |
|--------|---------|-------------|
| `hz` | Loop frequency | <15 Hz → reduce frame size / detection frequency |
| `status` | Threat level | Stays ALERT when no vehicle → adjust thresholds |
| `angle` | Vehicle bearing | Always 0° → check camera angles, focal length |
| `L`, `R` | Per-side TTR | Always equal → check `--angle-boost`, verify detections |
| Connection state | BLE link | "down" → check ESP32 power, signal strength |

## Getting Help

1. Check **VISION_SETUP.md** (comprehensive guide with calibration)
2. Review **tests/Camera_Testing/** (existing YOLO test scripts for reference)
3. Check camera provider tunables in **src/bicycle_blind_spot/vision/camera_provider.py**
4. Review threat algorithm in **src/Review/threat_algo.py**

---

**Ready? Run it:**
```bash
python scripts/ble_send_ttr_vision.py --headless false --visualize true
```
