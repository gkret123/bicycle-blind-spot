# bicycle-blind-spot

Python tooling for a bicycle blind-spot prototype that streams a normalized **TTR** (time-to-react) value to one or more BLE devices.

## What this repo contains

- `src/ble_client.py`: BLE client with auto-reconnect and optional notification handling.
- `src/ttr_source.py`: a placeholder TTR source (`TTRRamp`) that ramps from `1.0` down to `0.0`.
- `src/ttr_streamer.py`: pushes TTR updates to connected devices at a fixed rate.
- `src/metrics.py`: stream/reconnect metrics for terminal output.
- `scripts/ble_send_ttr.py`: CLI script that wires everything together.

## Requirements

- Python 3.11+
- A BLE-capable host (e.g., Raspberry Pi)
- Target BLE device(s) advertising names expected by the script (default: `BBSpot-XIAO`)

## Install

(Optional) Install system dependencies first (Debian/Ubuntu/Raspberry Pi OS):
```bash
sudo apt update
sudo xargs -a apt-requirements.txt apt install -y
```
> **Disclaimer:** `apt-requirements.txt` was generated from a development machine using `apt-mark showmanual`. Package availability and exact names may vary across Debian/Ubuntu/Raspberry Pi OS versions, so you may need to adjust the list for your specific system.


Then set up the Python environment:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

If you need all pinned dependencies used during development, you can also install:
```bash
pip install -r requirements.txt
```

## Run

Start the TTR streamer:

```bash
python scripts/ble_send_ttr.py --device-names "BBSpot-XIAO"
```

Common options:

- `--hz 20` update frequency
- `--ramp-s 45` ramp duration in seconds
- `--stop-at-end` stop once TTR reaches `0.0`
- `--notify` print BLE notifications from TX characteristic

Example with two targets:

```bash
python scripts/ble_send_ttr.py --device-names "BBSpot-XIAO-L,BBSpot-XIAO-R" --hz 25 --notify
```
## Raspberry Pi auto-start + graceful shutdown switch

This repository includes helper scripts for running the software at boot and shutting down cleanly when a physical switch is toggled OFF.

### Files

- `scripts/pi/run_bbs.sh`: wrapper that launches `scripts/ble_send_ttr.py` from `.venv`.
- `scripts/pi/power_switch_shutdown.py`: watches a GPIO pin and, when triggered, stops the main service then calls `shutdown -h now`.
- `scripts/pi/install_autostart.sh`: installs and enables both systemd services.
- `scripts/pi/systemd/bicycle-blind-spot.service`: boot service for the app.
- `scripts/pi/systemd/bicycle-power-switch.service`: boot service for the GPIO shutdown monitor.

### 1) Prepare your Pi environment

From the repository root:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

Install Raspberry Pi GPIO support if needed:

```bash
sudo apt update
sudo apt install -y python3-gpiozero
```

### 2) Install startup services

```bash
sudo scripts/pi/install_autostart.sh
```

This installs and enables:

- `bicycle-blind-spot.service` (main app at boot)
- `bicycle-power-switch.service` (GPIO switch monitor at boot)

### 3) Configure the shutdown switch pin

By default, the monitor uses BCM pin `17` and `--active-state low` (typical pull-up wiring where OFF pulls pin to GND).

If your wiring differs, edit:

- `scripts/pi/systemd/bicycle-power-switch.service`

Then reload and restart:

```bash
sudo systemctl daemon-reload
sudo systemctl restart bicycle-power-switch.service
```

### 4) Verify

```bash
systemctl status bicycle-blind-spot.service
systemctl status bicycle-power-switch.service
journalctl -u bicycle-power-switch.service -f
```

When the OFF switch is triggered, the monitor will:

1. `systemctl stop bicycle-blind-spot.service`
2. `shutdown -h now`

The app service uses `KillSignal=SIGINT` and a stop timeout so Python can run its disconnect logic before poweroff.

## Notes

- Current TTR generation is a synthetic ramp for integration testing.
- UUIDs and BLE defaults live in `src/constants.py` and should match firmware.
