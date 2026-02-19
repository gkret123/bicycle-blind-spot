"""
TTR Streamer.

Responsibilities:
- Pull TTR values from a TTR source (currently a ramp)
- Send TTR to one or more BLE clients at a configured update rate
- Print useful metrics (TTR, byte, send_hz, connection states, reconnect counts)
"""

from __future__ import annotations
import asyncio
from dataclasses import dataclass
from typing import Sequence, Optional, Dict

from .ble_client import BleAutoReconnectClient
from .metrics import StreamMetrics
from .math_utils import ttr_to_byte


@dataclass
class StreamConfig:
    hz: float = 20.0
    stop_at_end: bool = False


class TTRStreamer:
    def __init__(
        self,
        source,
        clients: Sequence[BleAutoReconnectClient],
        cfg: Optional[StreamConfig] = None,
        metrics: Optional[StreamMetrics] = None,
    ):
        self.source = source
        self.clients = list(clients)
        self.cfg = cfg or StreamConfig()
        self.metrics = metrics or StreamMetrics()

    async def start(self) -> None:
        # Ensure initial connections (in parallel)
        await asyncio.gather(*(c.ensure_connected() for c in self.clients))

        period = 1.0 / max(1e-6, self.cfg.hz)

        while True:
            ttr = self.source.value()
            b = ttr_to_byte(ttr)

            # send to all devices (in parallel)
            await asyncio.gather(*(c.send_ttr_scalar(ttr, response=False) for c in self.clients))
            self.metrics.note_write(len(self.clients))

            # build connection state summary for terminal
            states: Dict[str, str] = {}
            for c in self.clients:
                states[c.target.device_name] = "ok" if c.is_connected else "down"

            self.metrics.maybe_print(ttr=ttr, byte=b, states=states)

            if self.cfg.stop_at_end and ttr <= 0.0:
                break

            await asyncio.sleep(period)

        # At end, send "calm" (TTR=1.0) so firmware goes idle/silent
        await asyncio.gather(*(c.send_ttr_scalar(1.0, response=False) for c in self.clients))
