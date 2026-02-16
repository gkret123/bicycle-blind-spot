# src/ble_config.py
from __future__ import annotations
from dataclasses import dataclass

# GATT UUIDs (match ESP32 firmware)
SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
RX_UUID      = "12345678-1234-1234-1234-1234567890ac"
TX_UUID      = "12345678-1234-1234-1234-1234567890ad"

# Defaults
DEFAULT_DEVICE_NAMES = ["BBSpot-XIAO"]

# Scaling constants (data only)
TTR_BYTE_MIN = 0
TTR_BYTE_MAX = 255

@dataclass(frozen=True)
class BleTarget:
    device_name: str
    service_uuid: str = SERVICE_UUID
    rx_uuid: str = RX_UUID
    tx_uuid: str = TX_UUID
