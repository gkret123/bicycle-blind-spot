"""
BLE configuration and constants.

DATA ONLY
"""

from dataclasses import dataclass

# -------------------------------------------------------------------
# GATT UUIDs
# These MUST match what is defined in the ESP32 firmware.
# -------------------------------------------------------------------

SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
RX_UUID      = "12345678-1234-1234-1234-1234567890ac"  # we WRITE TTR here
TX_UUID      = "12345678-1234-1234-1234-1234567890ad"  # ESP32 NOTIFIES here


# -------------------------------------------------------------------
# TTR scaling constants
# These define how we convert a 0..1 float to a byte.
# -------------------------------------------------------------------

TTR_BYTE_MIN = 0
TTR_BYTE_MAX = 255


# -------------------------------------------------------------------
# Simple immutable container describing one BLE receiver target.
# Will create multiple of these (left/right motors later).
# -------------------------------------------------------------------

@dataclass(frozen=True)
class BleTarget:
    device_name: str
    service_uuid: str = SERVICE_UUID
    rx_uuid: str = RX_UUID
    tx_uuid: str = TX_UUID
