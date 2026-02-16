# src/ble_haptics.py
"""
BLE helper for BBSpot.

Primary use right now:
- Connect to ESP32 peripheral by advertised name
- Write 1-byte TTR scalar (0..1 -> 0..255) to RX characteristic

Dependencies:
    pip install bleak
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Optional, Callable, Awaitable

from bleak import BleakClient, BleakScanner


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


def ttr_to_byte(ttr: float) -> int:
    """
    Map TTR scalar [0,1] to a single byte [0,255].
    255 = calm (ttr=1.0), 0 = critical (ttr=0.0)
    """
    ttr = clamp01(ttr)
    b = int(round(ttr * 255.0))
    if b < 0:
        return 0
    if b > 255:
        return 255
    return b


@dataclass
class BleConfig:
    device_name: str
    service_uuid: str
    rx_uuid: str
    tx_uuid: Optional[str] = None
    scan_timeout_s: float = 8.0


class BleHapticsClient:
    """
    Minimal BLE client focused on:
      - find device by name
      - connect
      - write to RX characteristic (write without response by default)
      - optionally subscribe to notifications on TX
    """

    def __init__(self, cfg: BleConfig):
        self.cfg = cfg
        self._client: Optional[BleakClient] = None
        self._address: Optional[str] = None

    @property
    def is_connected(self) -> bool:
        return bool(self._client and self._client.is_connected)

    async def find(self) -> str:
        """
        Returns BLE address for the first device matching cfg.device_name.
        Raises RuntimeError if not found.
        """
        devices = await BleakScanner.discover(timeout=self.cfg.scan_timeout_s)
        for d in devices:
            if d.name == self.cfg.device_name:
                self._address = d.address
                return d.address
        raise RuntimeError(f"BLE device not found by name: {self.cfg.device_name!r}")

    async def connect(self) -> None:
        """
        Find (if needed) and connect.
        """
        if self._client and self._client.is_connected:
            return

        address = self._address or await self.find()
        client = BleakClient(address)
        await client.connect()
        self._client = client

    async def disconnect(self) -> None:
        """
        Disconnect if connected.
        """
        if self._client:
            try:
                if self._client.is_connected:
                    await self._client.disconnect()
            finally:
                self._client = None

    async def write(self, data: bytes, response: bool = False) -> None:
        """
        Write raw bytes to RX characteristic.
        """
        if not self._client or not self._client.is_connected:
            raise RuntimeError("BLE not connected")
        await self._client.write_gatt_char(self.cfg.rx_uuid, data, response=response)

    async def send_ttr_scalar(self, ttr: float, response: bool = False) -> None:
        """
        Convenience: send 1-byte TTR scalar.
        """
        b = ttr_to_byte(ttr)
        await self.write(bytes([b]), response=response)

    async def start_notify(
        self,
        handler: Callable[[bytearray], None] | Callable[[bytearray], Awaitable[None]],
    ) -> None:
        """
        Subscribe to notifications on TX characteristic (if provided).
        Handler may be sync or async.
        """
        if not self.cfg.tx_uuid:
            raise RuntimeError("tx_uuid not set in BleConfig")
        if not self._client or not self._client.is_connected:
            raise RuntimeError("BLE not connected")

        async def _wrap(_: int, data: bytearray) -> None:
            out = handler(data)
            if asyncio.iscoroutine(out):
                await out

        await self._client.start_notify(self.cfg.tx_uuid, _wrap)

    async def stop_notify(self) -> None:
        """
        Unsubscribe from notifications on TX characteristic (if provided).
        """
        if not self.cfg.tx_uuid:
            return
        if not self._client or not self._client.is_connected:
            return
        await self._client.stop_notify(self.cfg.tx_uuid)
