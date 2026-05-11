# bicycle-blind-spot

Python system for a bicycle blind-spot prototype that detects approaching vehicles and streams directional haptic warnings to BLE wearable devices.

The system computes a normalized **TTR** (Time-To-React) scalar — `0.0` = urgent, `1.0` = calm — from one or more sensor backends and broadcasts it to left/right haptic modules worn by the cyclist. Bearing angle from the threat determines which device vibrates (or both), giving the cyclist directional awareness.

---

## How it works

```
Sensor Backend
 ┌─────────────────────────────────────────────────────────────┐
 │  Vision:  Dual Pi cameras → YOLOv8 → TTC proxy             │
 │  Radar:   TI mmWave serial → TLV parse → Kalman tracker     │
 │  Fusion:  Both sensors → track association → blended urgency│
 │  Ramp:    Synthetic time ramp (integration testing only)    │
 └──────────────────────────┬──────────────────────────────────┘
                            │ .value() / .left_ttr / .right_ttr
                            ▼
                       TTRStreamer  (fixed Hz asyncio loop)
                            │
              ┌─────────────┴────────────┐
              ▼                          ▼
   BleAutoReconnectClient         BleAutoReconnectClient
      BBSpot-XIAO-L                  BBSpot-XIAO-R
        (left)                         (right)
              │                          │
              ▼                          ▼
   ESP32 + DRV2605 + LRA        ESP32 + DRV2605 + LRA
      (left wrist)                 (right wrist)
```

**Urgency model (all backends):**

```
urgency = w_proximity * proximity + w_closing * closing_speed
TTR     = 1.0 − clamp01(urgency)

approach gate: if closing_speed < threshold → TTR = 1.0 (no vibration)

side split:
  angle < −both_zone_deg  →  left only
  |angle| ≤ both_zone_deg →  both sides
  angle > +both_zone_deg  →  right only

EMA smoothing applied to each side's TTR for gradual transitions
```

---

## Repository layout

```
bicycle-blind-spot/
│
├── src/bicycle_blind_spot/
│   ├── ble/
│   │   ├── ble_client.py          # BLE auto-reconnect + GATT I/O
│   │   └── ttr_streamer.py        # Fixed-Hz broadcaster to multiple devices
│   │
│   ├── vision/
│   │   ├── camera_provider.py     # Dual Picamera2 + YOLOv8 + botsort tracking
│   │   └── ttr_source_vision.py   # Vision → TTR (bbox height + growth rate)
│   │
│   ├── radar/
│   │   ├── radar_provider.py      # TI mmWave serial → TLV → DBSCAN → Kalman
│   │   ├── ttr_source_radar.py    # Radar → TTR (range + closing speed)
│   │   └── ttr_source.py          # TTRRamp (synthetic, for testing)
│   │
│   ├── fusion/
│   │   ├── track_associator.py    # Match radar/vision by bearing angle
│   │   └── ttr_source_fusion.py   # Confidence-weighted + track-level fusion
│   │
│   ├── pi/
│   │   └── power_switch_shutdown.py  # GPIO pin watcher for graceful shutdown
│   │
│   ├── haptics/                   # Reserved
│   └── utils/
│       ├── constants.py           # GATT UUIDs, TTR byte scaling, BleTarget
│       ├── math_utils.py          # clamp, ttr_to_byte, byte_to_ttr
│       └── metrics.py             # StreamMetrics — writes/Hz/reconnects logging
│
├── scripts/
│   ├── ble_send_ttr.py            # Ramp mode (no sensors)
│   ├── ble_send_ttr_vision.py     # Vision mode
│   ├── ble_send_ttr_radar.py      # Radar mode
│   ├── ble_send_ttr_fusion.py     # Fusion mode (vision + radar)
│   ├── test_vision_only.py        # Vision smoke-test without BLE
│   └── pi/
│       ├── run_bbs.sh             # Wrapper for systemd ExecStart
│       ├── install_autostart.sh   # Install + enable both systemd services
│       ├── power_switch_shutdown.py
│       └── systemd/
│           ├── bicycle-blind-spot.service
│           └── bicycle-power-switch.service
│
├── firmware/
│   ├── esp32_BBSpot_Reciever/           # Single-device firmware
│   ├── esp32_BBSpot_Reciever_Left/      # Left-device firmware
│   └── esp32_BBSpot_Reciever_Right/     # Right-device firmware
│
├── yolov8n.pt                     # YOLOv8 nano weights (6.5 MB)
├── requirements.txt               # Pinned development dependencies
├── apt-requirements.txt           # System-level packages (Debian/Ubuntu/Pi OS)
├── pyproject.toml                 # Package metadata (Python 3.11+)
├── QUICKSTART_VISION.md           # Quick-start for the vision system
└── VISION_SETUP.md                # Camera calibration and mounting guide
```

---

## Hardware

### All modes
- Raspberry Pi with Bluetooth (tested on Pi 5)
- Two ESP32 BLE receiver boards (see `firmware/`) with:
  - Adafruit DRV2605 haptic driver
  - Linear resonant actuator (LRA) or vibration motor
- Default device names: `BBSpot-XIAO-L` (left), `BBSpot-XIAO-R` (right)

### Vision mode (additional)
- Two Raspberry Pi Camera Module v2.1 (or compatible)
- **Recommended mount — V-configuration:**
  - Left camera: −35° from straight-back
  - Right camera: +35° from straight-back
  - ~20 cm baseline between camera centers
  - Result: ~140° combined rear coverage with natural L/R assignment
- `--show` requires a display server (`$DISPLAY` or `$WAYLAND_DISPLAY`); headless SSH sessions auto-disable camera windows unless `BBS_FORCE_GUI=1` is set

### Radar mode (additional)
- TI mmWave radar module (IWR1443, IWR1642, or compatible)
- FTDI FT2232 dual-port USB serial adapter (auto-detected via VID:PID `0403:6010`)
- Two serial ports: CLI/config (`ttyUSB0`) and data (`ttyUSB1`)
- Two range presets:
  - `standard` — slope 40 MHz/μs, max range ~27 m
  - `long` (default) — slope 12 MHz/μs, max range ~90 m

---

## Install

Install system dependencies (Debian/Ubuntu/Raspberry Pi OS):
```bash
sudo apt update
sudo xargs -a apt-requirements.txt apt install -y
```
> `apt-requirements.txt` was generated via `apt-mark showmanual` on a development machine. Package names may vary across OS versions — adjust as needed.

Set up the Python environment:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

To reproduce the exact pinned development environment:
```bash
pip install -r requirements.txt
```

---

## Run

### Ramp mode — no sensors, integration testing only

Streams a synthetic linear ramp (`1.0 → 0.0`) over `--ramp-s` seconds. Useful for verifying BLE connectivity and firmware response without any sensor hardware.

```bash
python scripts/ble_send_ttr.py \
  --left-device "BBSpot-XIAO-L" \
  --right-device "BBSpot-XIAO-R" \
  --hz 20 \
  --ramp-s 45
```

| Flag | Default | Description |
|---|---|---|
| `--left-device` | `BBSpot-XIAO-L` | BLE name of left device |
| `--right-device` | `BBSpot-XIAO-R` | BLE name of right device |
| `--scan-timeout` | `8.0` | BLE scan timeout (seconds) |
| `--hz` | `20.0` | BLE update rate |
| `--ramp-s` | `45.0` | Ramp duration (seconds) |
| `--hold-s` | `10.0` | Hold at end value after ramp |
| `--stop-at-end` | off | Stop once TTR reaches 0.0 |
| `--notify` | off | Print TX notifications from devices |
| `--left-scale` | `1.0` | Multiply base TTR before sending to left device |
| `--left-offset` | `0.0` | Add offset to left TTR after scaling |
| `--right-scale` | `1.0` | Same for right device |
| `--right-offset` | `0.0` | Same for right device |
| `--print-side-values` | off | Print per-device TTR and byte each loop |
| `--print-every` | `0.5` | Metrics print interval (seconds) |

---

### Vision mode — dual Pi cameras + YOLOv8

Runs YOLO tracking on both cameras in a background thread. Urgency is computed from:
- **Proximity:** bounding-box height / frame height (vehicle size as range proxy)
- **Closing speed:** bounding-box height growth rate (px/s, EMA-smoothed)

```bash
# Headless / production
python scripts/ble_send_ttr_vision.py

# With OpenCV visualization windows
python scripts/ble_send_ttr_vision.py --show

# Test vision without BLE (smoke test)
python scripts/test_vision_only.py
```

| Flag | Default | Description |
|---|---|---|
| `--cam-left` / `--cam-right` | `0` / `1` | Picamera2 device indices |
| `--cam-angle` | `35.0` | Mount angle from straight-back (degrees); applied as −angle/+angle |
| `--show` | off | Show `cv2.imshow` windows (requires display server) |
| `--left-device` / `--right-device` | `BBSpot-XIAO-L/R` | BLE device names |
| `--scan-timeout` | `8.0` | BLE scan timeout |
| `--notify` | off | Print TX notifications |
| `--hz` | `20.0` | BLE update rate |
| `--print-every` | `0.5` | Metrics interval |
| `--w-proximity` | `0.4` | Weight for bbox-height urgency component |
| `--w-closing` | `0.6` | Weight for bbox-growth-rate urgency component |
| `--h-far` | `0.05` | Bbox height ratio below which proximity urgency = 0 |
| `--h-close` | `0.45` | Bbox height ratio above which proximity urgency = 1 |
| `--dhdt-max` | `40.0` | Bbox growth rate (px/s) mapped to maximum closing urgency |
| `--min-approach-dhdt` | `0.5` | Growth rate below which no vibration occurs (approach gate) |
| `--ttr-smooth` | `0.15` | EMA alpha for smoothing TTR output |
| `--both-zone` | `10.0` | ±degrees from center where both sides vibrate |
| `--print-side-values` | off | Print per-device TTR each loop |

**Vision internals:**
- Frame size: 512×288 px at 30 FPS target
- YOLO model: `yolov8n.pt` (nano), inference at 416 px
- Tracker: `botsort.yaml`, detects classes: car (2), motorcycle (3), bus (5), truck (7)
- Detection runs every 2nd frame; tracker propagates on skipped frames
- TTC seeded from first valid measurement (bypasses 99 s default to converge instantly)
- Status hysteresis: 5 consecutive detections to enter WARN/ALERT, 10 to leave

---

### Radar mode — TI mmWave sensor

Reads the serial data stream from the radar, processes it through the full pipeline, and computes urgency from:
- **Proximity:** inverted range in meters
- **Closing speed:** fused estimate — `0.65 × Doppler radial velocity + 0.35 × Kalman range rate`

```bash
# Default (long-range preset, ~90 m, auto-detect serial ports)
python scripts/ble_send_ttr_radar.py

# Standard-range preset (~27 m)
python scripts/ble_send_ttr_radar.py --range-preset standard

# If radar reports approaching vehicles as negative v_r
python scripts/ble_send_ttr_radar.py --approaching-sign -1

# Show live top-down radar scatter plot
python scripts/ble_send_ttr_radar.py --show
```

| Flag | Default | Description |
|---|---|---|
| `--cfg-port` | `auto` | Radar CLI serial port (auto-detects FTDI FT2232) |
| `--data-port` | `auto` | Radar data serial port |
| `--range-preset` | `long` | `standard` (~27 m) or `long` (~90 m) |
| `--approaching-sign` | `1` | Sign convention for approaching radial velocity (`+1` or `−1`) |
| `--left-device` / `--right-device` | `BBSpot-XIAO-L/R` | BLE device names |
| `--scan-timeout` | `8.0` | BLE scan timeout |
| `--hz` | `20.0` | BLE update rate |
| `--print-every` | `0.5` | Metrics interval |
| `--w-proximity` | `0.4` | Weight for range-based urgency component |
| `--w-closing` | `0.6` | Weight for closing-speed urgency component |
| `--range-far` | `80.0` | Range (m) beyond which proximity urgency = 0 |
| `--range-close` | `3.0` | Range (m) within which proximity urgency = 1 |
| `--closing-max` | `15.0` | Closing speed (m/s) mapped to maximum urgency |
| `--min-approach-mps` | `0.5` | Closing speed below which no vibration occurs |
| `--ttr-smooth` | `0.15` | EMA smoothing alpha |
| `--both-zone` | `10.0` | ±degrees from center where both sides vibrate |
| `--show` | off | Live top-down matplotlib scatter plot |
| `--verbose-radar` | off | Print per-frame radar pipeline summaries |
| `--print-side-values` | off | Print per-device TTR each loop |

**Radar pipeline:**
```
Serial TLV packets → parse point cloud (TLV type 1, Nx4 float32 [x,y,z,v_r])
  → ROI filter (lateral ±3 m, range 1.5–100 m, height ±2.5 m)
  → DBSCAN clustering (eps=0.9 m, min_samples=4)
  → vehicle-like filter (size + velocity std)
  → Kalman multi-tracker (predict → update → prune stale)
  → best-target selection (approaching-first scoring)
  → RadarResult (range_m, closing_mps, angle_deg, n_tracks)
```

Serial reconnection handles EMI-triggered USB glitches automatically.

---

### Fusion mode — vision + radar combined

Runs both sensor pipelines in parallel background threads. A third fusion thread reads the latest result from each, optionally associates them by bearing angle, and blends urgency using dynamic weights that adapt by range:

| Range | Radar weight | Camera weight |
|---|---|---|
| > 15 m | 0.85 | 0.15 |
| 5–15 m | linear crossfade | linear crossfade |
| < 5 m | 0.15 | 0.85 |

Weights are further scaled by per-sensor confidence (radar: track hit count; camera: YOLO detection score + closing speed). If one sensor drops out, the other takes full weight.

```bash
# Headless / production
python scripts/ble_send_ttr_fusion.py

# Enable both visualization windows
python scripts/ble_send_ttr_fusion.py --show

# Camera windows only
python scripts/ble_send_ttr_fusion.py --cam-show

# Radar top-down plot only
python scripts/ble_send_ttr_fusion.py --radar-show

# Use track-level association (more robust, requires bearing match)
python scripts/ble_send_ttr_fusion.py --fusion-strategy track_level
```

| Flag | Default | Description |
|---|---|---|
| `--cam-left` / `--cam-right` | `0` / `1` | Camera device indices |
| `--cam-angle` | `35.0` | Camera mount angle from straight-back |
| `--cam-show` | off | Show camera windows |
| `--cfg-port` / `--data-port` | `auto` | Radar serial ports |
| `--range-preset` | `long` | Radar range preset |
| `--approaching-sign` | `1` | Radar sign convention |
| `--radar-show` | off | Show radar scatter plot |
| `--verbose-radar` | off | Radar per-frame summaries |
| `--show` | off | Enable all visualizations |
| `--left-device` / `--right-device` | `BBSpot-XIAO-L/R` | BLE device names |
| `--hz` | `20.0` | BLE update rate |
| `--near-range` | `5.0` | Below this (m), camera dominates |
| `--far-range` | `15.0` | Above this (m), radar dominates |
| `--angle-gate` | `12.0` | Max bearing difference (deg) to associate tracks |
| `--max-time-delta` | `0.3` | Max time gap (s) between sensor readings for association |
| `--fusion-strategy` | `confidence_weighted` | `confidence_weighted` or `track_level` |
| `--ttr-smooth` | `0.15` | EMA smoothing alpha |
| `--both-zone` | `10.0` | ±degrees for bilateral vibration |
| `--h-far`, `--h-close`, `--dhdt-max`, `--min-approach-dhdt` | (same as vision) | Vision urgency tuning |
| `--range-far`, `--range-close`, `--closing-max`, `--min-approach-mps` | (same as radar) | Radar urgency tuning |
| `--print-side-values` | off | Per-device TTR each loop |

**Fusion strategies:**
- `confidence_weighted` (recommended): blends urgency from the best-threat of each sensor independently, weighted by range + confidence
- `track_level`: uses `TrackAssociator` to match radar and camera detections by bearing before computing fused urgency; produces `FUSED`, `RADAR`, or `VISION` labels in terminal output

---

## BLE protocol

GATT UUIDs are defined in [src/bicycle_blind_spot/utils/constants.py](src/bicycle_blind_spot/utils/constants.py) and **must match the ESP32 firmware**:

| Role | UUID |
|---|---|
| Service | `12345678-1234-1234-1234-1234567890ab` |
| RX (host writes TTR here) | `12345678-1234-1234-1234-1234567890ac` |
| TX (device notifies here) | `12345678-1234-1234-1234-1234567890ad` |

**Encoding:** TTR float `[0, 1]` → single byte `[0, 255]` via `ttr_to_byte()`. `255` = calm, `0` = maximum urgency.

**Auto-reconnect:** `BleAutoReconnectClient` uses exponential backoff (initial 0.5 s, max 8 s). On disconnect mid-stream, it reconnects and retries the write. On clean shutdown, a final TTR=1.0 (calm) is sent to all devices so firmware goes idle.

**Per-device transforms:** `TTRStreamer` accepts an optional `transforms` dict mapping device name → `(float) → float`. The ramp script uses this to apply independent `scale` and `offset` per side. The vision/radar/fusion scripts instead inject the angle-split TTR directly as a transform, bypassing the base TTR entirely for per-device shaping.

**TX notifications:** pass `--notify` to any script to print raw bytes received from the device's TX characteristic. Notifications are re-subscribed automatically after reconnects.

---

## Raspberry Pi auto-start + graceful shutdown

### Install startup services

```bash
sudo scripts/pi/install_autostart.sh
```

Installs and enables two systemd services:
- `bicycle-blind-spot.service` — main application, launched at boot via `scripts/pi/run_bbs.sh`
- `bicycle-power-switch.service` — GPIO shutdown monitor, launched at boot

### Configure the shutdown switch

Default: BCM GPIO pin `17`, `--active-state low` (pull-up wiring — switch OFF pulls pin to GND).

To change the pin, edit `scripts/pi/systemd/bicycle-power-switch.service`, then:
```bash
sudo systemctl daemon-reload
sudo systemctl restart bicycle-power-switch.service
```

### Verify

```bash
systemctl status bicycle-blind-spot.service
systemctl status bicycle-power-switch.service
journalctl -u bicycle-blind-spot.service -f
```

When the OFF switch triggers, the monitor:
1. Runs `systemctl stop bicycle-blind-spot.service`
2. Runs `shutdown -h now`

The app service uses `KillSignal=SIGINT` so Python's `finally` block sends a calm signal and disconnects cleanly before poweroff.

---

## Firmware

Three Arduino sketches in `firmware/` for the ESP32 haptic receiver. Flash the left and right variants to their respective boards.

Each device:
- Advertises its BLE name (`BBSpot-XIAO-L` or `BBSpot-XIAO-R`)
- Exposes a GATT service with RX (writable) and TX (notify) characteristics matching the UUIDs above
- Drives a DRV2605 haptic controller in response to received TTR bytes

---

## Additional documentation

- [QUICKSTART_VISION.md](QUICKSTART_VISION.md) — quick-start for the dual-camera vision system, test scenarios, console output reference, troubleshooting
- [VISION_SETUP.md](VISION_SETUP.md) — camera mounting configurations, focal length calibration, stereo baseline measurement, detailed troubleshooting
