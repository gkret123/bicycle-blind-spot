# src/ble_client.py
"""
BLE Auto-Reconnect Client.

Responsibilities:
- Find BLE device by name
- Connect
- Auto-reconnect if connection drops
- Send TTR byte
- Subscribe to TX notifications

This file contains ALL BLE behavior logic.
"""

import asyncio
from dataclasses import dataclass
from typing import Optional, Callable, Awaitable

from bleak import BleakClient, BleakScanner

from .constants import BleTarget
from .math_utils import ttr_to_byte


# Type alias for notification callback
NotifyHandler = Callable[[bytes], None] | Callable[[bytes], Awaitable[None]]


@dataclass
class ReconnectPolicy:
    """
    Controls reconnect timing behavior.
    """
    scan_timeout_s: float = 8.0
    initial_backoff_s: float = 0.5
    max_backoff_s: float = 8.0


class BleAutoReconnectClient:
    """
    High-level BLE client that automatically reconnects.
    """

    def __init__(self, target: BleTarget, policy: Optional[ReconnectPolicy] = None):
        self.target = target
        self.policy = policy or ReconnectPolicy()

        self._client: Optional[BleakClient] = None
        self._address: Optional[str] = None

        self._notify_handler: Optional[NotifyHandler] = None
        self._notify_enabled = False

        self._lock = asyncio.Lock()

    # ---------------------------
    # Connection Handling
    # ---------------------------

    async def _find_address(self) -> str:
        """
        Scan for device by name and return its address.
        """
        devices = await BleakScanner.discover(timeout=self.policy.scan_timeout_s)
        for d in devices:
            if d.name == self.target.device_name:
                return d.address
        raise RuntimeError(f"Device not found: {self.target.device_name}")

    async def connect(self):
        """
        Connect once (no retry loop).
        """
        async with self._lock:
            if self._client and self._client.is_connected:
                return

            self._address = await self._find_address()
            self._client = BleakClient(self._address)
            await self._client.connect()

            # Re-enable notifications after reconnect
            if self._notify_handler:
                await self._start_notify_locked(self._notify_handler)

    async def ensure_connected(self):
        """
        Keep retrying until connected.
        """
        backoff = self.policy.initial_backoff_s
        while True:
            try:
                await self.connect()
                return
            except Exception:
                await asyncio.sleep(backoff)
                backoff = min(self.policy.max_backoff_s, backoff * 2)

    async def disconnect(self):
        if self._client and self._client.is_connected:
            await self._client.disconnect()
        self._client = None

    # ---------------------------
    # Writing Data
    # ---------------------------

    async def write(self, data: bytes, response: bool = False):
        await self.ensure_connected()
        try:
            await self._client.write_gatt_char(self.target.rx_uuid, data, response=response)
        except Exception:
            # Try reconnect once
            await self.disconnect()
            await self.ensure_connected()
            await self._client.write_gatt_char(self.target.rx_uuid, data, response=response)

    async def send_ttr_scalar(self, ttr: float):
        """
        Convert TTR to byte and send it.
        """
        byte = ttr_to_byte(ttr)
        await self.write(bytes([byte]))

    # ---------------------------
    # Notifications
    # ---------------------------

    async def start_notify(self, handler: NotifyHandler):
        """
        Subscribe to TX notifications.
        """
        self._notify_handler = handler
        await self.ensure_connected()
        await self._start_notify_locked(handler)

    async def _start_notify_locked(self, handler: NotifyHandler):
        async def wrapper(_, data: bytearray):
            result = handler(bytes(data))
            if asyncio.iscoroutine(result):
                await result

        await self._client.start_notify(self.target.tx_uuid, wrapper)
        self._notify_enabled = True
