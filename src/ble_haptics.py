"""BLE transport and compact command protocol for Pi -> ESP32 haptics control.

This module defines:
- A compact 4-byte command packet for low-rate control updates.
- An asyncio BLE client wrapper for connecting/writing with ``bleak``.

Packet layout (4 bytes):
    b0: intensity (0-255)
    b1: pattern id (0-255)
    b2: duration in 10 ms ticks (0-255 => 0..2550 ms)
    b3: side id (0=both/center, 1=left, 2=right)
"""

#use: python radar_test.py --cfg COM4 --data COM5

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


DEFAULT_DEVICE_NAME = "BBSpot-XIAO"
DEFAULT_SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
DEFAULT_RX_UUID = "12345678-1234-1234-1234-1234567890ac"


class HapticSide(IntEnum):
    BOTH = 0
    LEFT = 1
    RIGHT = 2


class HapticPattern(IntEnum):
    OFF = 0
    AMPLITUDE_PULSE = 1
    RATE_PULSE = 2
    CUSTOM = 255


@dataclass(frozen=True)
class HapticCommand:
    """Compact haptic command to send from Pi to ESP32."""

    intensity: int
    pattern: HapticPattern
    duration_ms: int
    side: HapticSide = HapticSide.BOTH

    def to_payload(self) -> bytes:
        """Serialize command into the compact 4-byte wire format."""
        intensity = _clamp(self.intensity, 0, 255)
        pattern = _clamp(int(self.pattern), 0, 255)
        ticks_10ms = _clamp(round(self.duration_ms / 10.0), 0, 255)
        side = _clamp(int(self.side), 0, 255)
        return bytes((intensity, pattern, ticks_10ms, side))


class BleHapticsClient:
    """Small BLE helper for command delivery to ESP32."""

    def __init__(
        self,
        *,
        device_name: str = DEFAULT_DEVICE_NAME,
        service_uuid: str = DEFAULT_SERVICE_UUID,
        rx_uuid: str = DEFAULT_RX_UUID,
        scan_timeout_s: float = 8.0,
    ) -> None:
        self.device_name = device_name
        self.service_uuid = service_uuid
        self.rx_uuid = rx_uuid
        self.scan_timeout_s = scan_timeout_s
        self._address: Optional[str] = None
        self._client = None

    async def connect(self) -> None:
        if self._client and self._client.is_connected:
            return

        if self._address is None:
            self._address = await self._find_device_address()

        from bleak import BleakClient

        client = BleakClient(self._address)
        await client.connect()
        if not client.is_connected:
            raise RuntimeError(f"Failed to connect to {self._address}")
        self._client = client

    async def disconnect(self) -> None:
        if self._client:
            await self._client.disconnect()
        self._client = None

    async def send(self, cmd: HapticCommand, *, response: bool = False) -> None:
        if not self._client or not self._client.is_connected:
            raise RuntimeError("BLE client not connected")
        await self._client.write_gatt_char(self.rx_uuid, cmd.to_payload(), response=response)

    async def send_and_wait(self, cmd: HapticCommand, wait_s: float) -> None:
        await self.send(cmd)
        await asyncio.sleep(wait_s)

    async def _find_device_address(self) -> str:
        expected_name = self.device_name
        expected_service = self.service_uuid.lower()

        def _matches(device, adv) -> bool:
            adv_uuids = getattr(adv, "service_uuids", None) or []
            if expected_service in [u.lower() for u in adv_uuids]:
                return True
            name = device.name or ""
            return name == expected_name

        from bleak import BleakScanner

        dev = await BleakScanner.find_device_by_filter(_matches, timeout=self.scan_timeout_s)
        if dev is None:
            raise RuntimeError(
                f"Could not find BLE device '{self.device_name}' with service '{self.service_uuid}'"
            )
        return dev.address


def _clamp(value: int, lo: int, hi: int) -> int:
    return lo if value < lo else hi if value > hi else value
