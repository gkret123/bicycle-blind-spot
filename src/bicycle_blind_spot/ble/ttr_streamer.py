"""
TTR Streamer.

Responsibilities:
- Pull TTR values from a TTR source (currently a ramp)
- Optionally transform the base TTR per client/device
- Send TTR to one or more BLE clients at a configured update rate
- Print useful metrics (TTR, byte, send_hz, connection states, reconnect counts)
- Optionally expose per-device TTR details for debugging

This version supports:
- per-device transforms keyed by device name
- logical labels for cleaner terminal output, e.g.:
    left:BBSpot-XIAO-L
    right:BBSpot-XIAO-R
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Sequence

from .ble_client import BleAutoReconnectClient
from bicycle_blind_spot.utils.metrics import StreamMetrics
from bicycle_blind_spot.utils.math_utils import ttr_to_byte


TTRTransform = Callable[[float], float]


@dataclass
class StreamConfig:
    hz: float = 20.0
    stop_at_end: bool = False
    print_per_client_values: bool = False


class TTRStreamer:
    def __init__(
        self,
        source,
        clients: Sequence[BleAutoReconnectClient],
        cfg: Optional[StreamConfig] = None,
        metrics: Optional[StreamMetrics] = None,
        transforms: Optional[Dict[str, TTRTransform]] = None,
        labels: Optional[Dict[str, str]] = None,
    ):
        """
        Args:
            source:
                Any object with a .value() -> float method returning the base TTR.

            clients:
                BLE clients to stream to.

            cfg:
                Stream configuration.

            metrics:
                Metrics printer/collector.

            transforms:
                Optional mapping:
                    client.target.device_name -> transform(base_ttr) -> per-client ttr

                If absent for a client, identity is used.

                Example:
                    {
                        "BBSpot-XIAO-L": left_cfg.apply,
                        "BBSpot-XIAO-R": right_cfg.apply,
                    }

            labels:
                Optional mapping:
                    client.target.device_name -> display label

                Used for terminal output and per-client debug printing.

                Example:
                    {
                        "BBSpot-XIAO-L": "left:BBSpot-XIAO-L",
                        "BBSpot-XIAO-R": "right:BBSpot-XIAO-R",
                    }
        """
        self.source = source
        self.clients = list(clients)
        self.cfg = cfg or StreamConfig()
        self.metrics = metrics or StreamMetrics()
        self.transforms = transforms or {}
        self.labels = labels or {}

    def _transform_for_client(self, client: BleAutoReconnectClient, base_ttr: float) -> float:
        fn = self.transforms.get(client.target.device_name)
        if fn is None:
            return base_ttr
        return fn(base_ttr)

    def _label_for_client(self, client: BleAutoReconnectClient) -> str:
        return self.labels.get(client.target.device_name, client.target.device_name)

    async def start(self) -> None:
        # Ensure initial connections in parallel.
        await asyncio.gather(*(c.ensure_connected() for c in self.clients))

        period = 1.0 / max(1e-6, self.cfg.hz)

        try:
            while True:
                base_ttr = self.source.value()
                base_byte = ttr_to_byte(base_ttr)

                # Compute per-client TTR values first so they can be both sent and printed.
                per_client_ttr: Dict[str, float] = {}
                for client in self.clients:
                    per_client_ttr[client.target.device_name] = self._transform_for_client(client, base_ttr)

                # Send to all devices in parallel.
                await asyncio.gather(
                    *(
                        client.send_ttr_scalar(
                            per_client_ttr[client.target.device_name],
                            response=False,
                        )
                        for client in self.clients
                    )
                )
                self.metrics.note_write(len(self.clients))

                # Build connection state summary for terminal.
                states: Dict[str, str] = {}
                for client in self.clients:
                    states[self._label_for_client(client)] = "ok" if client.is_connected else "down"

                self.metrics.maybe_print(
                    ttr=base_ttr,
                    byte=base_byte,
                    states=states,
                )

                if self.cfg.print_per_client_values:
                    detail = " ".join(
                        f"{self._label_for_client(client)}=("
                        f"{per_client_ttr[client.target.device_name]:.3f}/"
                        f"{ttr_to_byte(per_client_ttr[client.target.device_name])})"
                        for client in self.clients
                    )
                    print(f"[signal] base=({base_ttr:.3f}/{base_byte}) {detail}")

                if self.cfg.stop_at_end and base_ttr <= 0.0:
                    break

                await asyncio.sleep(period)

        finally:
            # At end, send "calm" (TTR=1.0) so firmware goes idle/silent.
            await asyncio.gather(
                *(client.send_ttr_scalar(1.0, response=False) for client in self.clients),
                return_exceptions=True,
            )