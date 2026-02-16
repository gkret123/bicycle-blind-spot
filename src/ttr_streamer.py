# src/ttr_streamer.py
"""
TTR Streamer.

Connects one TTR source to one or more BLE clients.
"""

import asyncio
from dataclasses import dataclass
from typing import Sequence

from .ble_client import BleAutoReconnectClient
from .ttr_source import TTRRamp


@dataclass
class StreamConfig:
    """Configuration for the streaming behavior."""
    hz: float = 20.0  # Streaming frequency in hertz
    stop_at_end: bool = False  # Whether to stop streaming when source value reaches zero


class TTRStreamer:
    """Streams TTR values from a source to multiple BLE clients."""
    def __init__(self, source: TTRRamp, clients: Sequence[BleAutoReconnectClient], cfg: StreamConfig):
        """Initialize the streamer with a source, clients, and configuration."""
        self.source = source
        self.clients = list(clients)
        self.cfg = cfg

    async def start(self):
        """Start streaming TTR values to all connected clients."""
        # Ensure all BLE clients are connected before streaming
        await asyncio.gather(*(c.ensure_connected() for c in self.clients))

        # Calculate the time interval between transmissions based on frequency
        period = 1.0 / self.cfg.hz

        # Main streaming loop
        while True:
            # Get the current TTR value from the source
            value = self.source.value()

            # Send the value to all connected clients concurrently
            await asyncio.gather(*(c.send_ttr_scalar(value) for c in self.clients))

            # Exit the loop if configured to stop when value drops to zero
            if self.cfg.stop_at_end and value <= 0.0:
                break

            # Wait for the calculated period before the next transmission
            await asyncio.sleep(period)
