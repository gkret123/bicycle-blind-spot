# Bicycle Blind Spot

Python tooling for a bicycle blind-spot prototype that computes a **TTR** (time-to-react) signal from radar, dual-camera vision, or fused sensors and streams left/right haptic intensity over BLE.

## System modes

- **Vision (dual-camera + YOLO)** — `scripts/ble_send_ttr_vision.py`
- **Radar (TI mmWave)** — `scripts/ble_send_ttr_radar.py`
- **Fusion (radar + vision)** — `scripts/ble_send_ttr_fusion.py`
- **Synthetic ramp demo** — `scripts/ble_send_ttr.py`

## Repository layout

- `src/bicycle_blind_spot/ble/` — BLE client + streaming loop
- `src/bicycle_blind_spot/vision/` — dual-camera YOLO tracking + TTR source
- `src/bicycle_blind_spot/radar/` — mmWave parsing + TTR source
- `src/bicycle_blind_spot/fusion/` — radar/vision association + blending
- `scripts/` — CLI entry points for each mode
- `scripts/pi/` — Raspberry Pi systemd helpers + shutdown switch
- `QUICKSTART_VISION.md`, `VISION_SETUP.md` — camera mounting + calibration
- `tests/` — experimental scripts and data collection helpers

## Requirements

- Python **3.11+**
- BLE-capable Linux host (Raspberry Pi recommended)
- Two BLE haptic devices (defaults: `BBSpot-XIAO-L` and `BBSpot-XIAO-R`)
- **Vision mode:** two Raspberry Pi cameras + `yolov8n.pt` (included)
- **Radar mode:** TI mmWave radar + serial/FTDI adapter

## Install

(Optional) Install system dependencies first (Debian/Ubuntu/Raspberry Pi OS):
```bash
sudo apt update
sudo xargs -a apt-requirements.txt apt install -y
```
> **Note:** `apt-requirements.txt` was generated on a dev machine. Package names may vary across OS versions.

Create a virtual environment and install the package:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

If you need the full pinned development stack:
```bash
pip install -r requirements.txt
```

## Run

### Vision (dual-camera)
```bash
# Headless (default)
python scripts/ble_send_ttr_vision.py

# With camera windows
python scripts/ble_send_ttr_vision.py --show
```

Helpful options:
- `--cam-left`, `--cam-right` — camera indices
- `--cam-angle` — mount angle from straight-back (deg)
- `--both-zone` — angle zone where both sides vibrate
- `--w-proximity`, `--w-closing` — urgency weighting

### Radar (TI mmWave)
```bash
python scripts/ble_send_ttr_radar.py --range-preset long

# Show live radar scatter plot
python scripts/ble_send_ttr_radar.py --show
```

### Fusion (radar + vision)
```bash
# Headless (default)
python scripts/ble_send_ttr_fusion.py

# Enable camera + radar visualizations
python scripts/ble_send_ttr_fusion.py --show
```

### Synthetic ramp demo
```bash
python scripts/ble_send_ttr.py --ramp-s 45 --hold-s 10
```

### Vision-only test (no BLE required)
```bash
python scripts/test_vision_only.py
```

## Raspberry Pi autostart + shutdown switch

This repo includes systemd helpers for running the default BLE ramp streamer at boot and shutting down cleanly when a GPIO switch is toggled.

- `scripts/pi/run_bbs.sh` — boot wrapper that activates `.venv` and runs `ble_send_ttr.py`
- `scripts/pi/power_switch_shutdown.py` — GPIO monitor that stops the service, then powers off
- `scripts/pi/install_autostart.sh` — installs both systemd services

Install:
```bash
sudo scripts/pi/install_autostart.sh
```

To use vision/fusion, edit the installed service (for example,
`/etc/systemd/system/bicycle-blind-spot.service`, based on
`scripts/pi/systemd/bicycle-blind-spot.service`) and replace `ExecStart` with
the command you want to run. Example:
```ini
ExecStart=/home/pi/bicycle-blind-spot/scripts/pi/run_bbs.sh
```
Then reload and restart systemd.

## More documentation

- **QUICKSTART_VISION.md** — getting the dual-camera pipeline running quickly
- **VISION_SETUP.md** — detailed mounting, calibration, and troubleshooting

## License

See the repository for license details.
