# bicycle-blind-spot

Python tooling for a bicycle blind-spot prototype that streams a normalized **TTR** (time-to-react) value to one or more BLE devices.

## What this repo contains

- `src/bicycle_blind_spot/ble_client.py`: BLE client with auto-reconnect and optional notification handling.
- `src/bicycle_blind_spot/ttr_source.py`: a placeholder TTR source (`TTRRamp`) that ramps from `1.0` down to `0.0`.
- `src/bicycle_blind_spot/ttr_streamer.py`: pushes TTR updates to connected devices at a fixed rate.
- `src/bicycle_blind_spot/metrics.py`: stream/reconnect metrics for terminal output.
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


## Package import

After installing, import modules from the real package name:

```python
from bicycle_blind_spot.constants import BleTarget
from bicycle_blind_spot.ttr_source import TTRRamp
```

## What changed (plain English)

If you were previously doing imports like `from src.constants import ...`, that pattern was causing packaging/import issues.

The project now follows the standard Python layout:

- `src/` is only a folder that holds source code.
- `src/bicycle_blind_spot/` is the real Python package.
- Imports should use `bicycle_blind_spot`, not `src`.

In short:

- **Before:** `from src.constants import BleTarget`
- **Now:** `from bicycle_blind_spot.constants import BleTarget`

Why this is better:

- `pip install -e .` works in the normal/expected way.
- IDEs and tooling can find imports more reliably.
- You avoid `No module named 'src'` style errors.

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
- UUIDs and BLE defaults live in `src/bicycle_blind_spot/constants.py` and should match firmware.
