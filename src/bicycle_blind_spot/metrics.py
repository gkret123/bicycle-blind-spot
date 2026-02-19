"""
Streaming metrics / logging.

This module is ONLY responsible for tracking and printing metrics.
It does not know about BLE internals or how TTR is calculated.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict
import time


@dataclass
class StreamMetrics:
    # How often to print a status line
    print_every_s: float = 0.5

    started_at: float = field(default_factory=time.monotonic)
    last_print_at: float = field(default_factory=time.monotonic)

    # total number of BLE writes issued (across all devices)
    writes_total: int = 0

    # reconnect counter per device name
    reconnects: Dict[str, int] = field(default_factory=dict)

    def note_write(self, count: int = 1) -> None:
        self.writes_total += count

    def note_reconnect(self, device_name: str) -> None:
        self.reconnects[device_name] = self.reconnects.get(device_name, 0) + 1

    def maybe_print(self, *, ttr: float, byte: int, states: Dict[str, str]) -> None:
        now = time.monotonic()
        if now - self.last_print_at < self.print_every_s:
            return

        self.last_print_at = now
        elapsed = now - self.started_at
        hz = (self.writes_total / elapsed) if elapsed > 0 else 0.0

        devs = " ".join([f"{name}:{state}" for name, state in states.items()])
        reconn = " ".join([f"{k}={v}" for k, v in self.reconnects.items()]) or "-"

        print(
            f"t={elapsed:6.1f}s  ttr={ttr:0.3f}  bytes={byte:3d}  "
            f"send_hz={hz:0.1f}  devs=[{devs}]  reconn=[{reconn}]"
        )
