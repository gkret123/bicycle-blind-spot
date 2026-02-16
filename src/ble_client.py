# src/ble_client.py
"""
BLE Auto-Reconnect Client.

Responsibilities:
- Find BLE device by name
- Connect
- Auto-reconnect if connection drops
- Write bytes to RX characteristic
- Send TTR scalar as a single byte
- Subscribe to TX notifications (and keep it enabled across reconnects)

This module intentionally does NOT print metrics.
use src/metrics.py and src/ttr_streamer.py for logging.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Callable, Awaitable, Optional

from bleak import BleakClient, BleakScanner

from .constants import BleTarget  # (you renamed ble_config.py back to constants.py)
from .math_utils import ttr_to_byte


NotifyHandler = Callable[[bytes], None] | Callable[[bytes], Awaitable[None]]
ReconnectCallback = Callable[[str], None]  # receives device_name


@dataclass
class ReconnectPolicy:
    scan_timeout_s: float = 8.0
    initial_backoff_s: float = 0.5
    max_backoff_s: float = 8.0


class BleAutoReconnectClient:
    def __init__(
        self,
        target: BleTarget,
        policy: Optional[ReconnectPolicy] = None,
        on_reconnect: Optional[ReconnectCallback] = None,
    ):
        self.target = target
        self.policy = policy or ReconnectPolicy()
        self._on_reconnect = on_reconnect

        self._client: Optional[BleakClient] = None
        self._address: Optional[str] = None

        # notification state
        self._notify_handler: Optional[NotifyHandler] = None
        self._notify_enabled = False

        self._lock = asyncio.Lock()

    @property
    def is_connected(self) -> bool:
        return bool(self._client and self._client.is_connected)

    async def _find_address(self) -> str:
        devices = await BleakScanner.discover(timeout=self.policy.scan_timeout_s)
        for d in devices:
            if d.name == self.target.device_name:
                return d.address
        raise RuntimeError(f"BLE device not found: {self.target.device_name!r}")

    async def connect(self) -> None:
        """
        Try once to connect. No retry loop here (that's ensure_connected()).
        """
        async with self._lock:
            if self._client and self._client.is_connected:
                return

            self._address = await self._find_address()
            client = BleakClient(self._address)
            await client.connect()
            self._client = client

            # restore notifications on reconnect
            if self._notify_handler and not self._notify_enabled:
                await self._start_notify_locked(self._notify_handler)

    async def ensure_connected(self) -> None:
        """
        Keep retrying until connected (with exponential backoff).
        Calls on_reconnect(device_name) when a reconnect succeeds.
        """
        backoff = self.policy.initial_backoff_s
        had_failure = False

        while True:
            try:
                await self.connect()
                if had_failure and self._on_reconnect:
                    self._on_reconnect(self.target.device_name)
                return
            except Exception:
                had_failure = True
                await asyncio.sleep(backoff)
                backoff = min(self.policy.max_backoff_s, backoff * 2)

    async def disconnect(self) -> None:
        async with self._lock:
            if not self._client:
                return
            try:
                if self._notify_enabled:
                    try:
                        await self._client.stop_notify(self.target.tx_uuid)
                    except Exception:
                        pass
                    self._notify_enabled = False

                if self._client.is_connected:
                    await self._client.disconnect()
            finally:
                self._client = None

    async def write(self, data: bytes, response: bool = False) -> None:
        """
        Write raw bytes to RX characteristic.
        If write fails (disconnect), reconnect and retry once.
        """
        await self.ensure_connected()
        assert self._client is not None

        try:
            await self._client.write_gatt_char(self.target.rx_uuid, data, response=response)
        except Exception:
            await self.disconnect()
            await self.ensure_connected()
            assert self._client is not None
            await self._client.write_gatt_char(self.target.rx_uuid, data, response=response)

    async def send_ttr_scalar(self, ttr: float, response: bool = False) -> None:
        """
        Convert TTR (0..1) -> 1 byte and write it.
        """
        b = ttr_to_byte(ttr)
        await self.write(bytes([b]), response=response)

    async def start_notify(self, handler: NotifyHandler) -> None:
        """
        Enable TX notifications and keep them enabled across reconnects.
        """
        async with self._lock:
            self._notify_handler = handler
            if self._client and self._client.is_connected:
                await self._start_notify_locked(handler)

    async def _start_notify_locked(self, handler: NotifyHandler) -> None:
        assert self._client is not None

        async def _wrap(_: int, data: bytearray) -> None:
            out = handler(bytes(data))
            if asyncio.iscoroutine(out):
                await out

        await self._client.start_notify(self.target.tx_uuid, _wrap)
        self._notify_enabled = True
