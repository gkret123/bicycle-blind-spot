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

## Notes

- Current TTR generation is a synthetic ramp for integration testing.
- UUIDs and BLE defaults live in `src/constants.py` and should match firmware.
