# src/ttr_source.py
"""
TTR provider (0.0..1.0).
For now: ramps from 1.0 -> 0.0 over ramp_seconds, then holds at 0.0.
Later: replace with real TTR computed from range + rel vel.
"""

from __future__ import annotations
from dataclasses import dataclass
import time


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


@dataclass
class TTRRamp:
    ramp_seconds: float = 45.0
    hold_seconds: float = 10.0  # currently informational; we just keep output at 0 after ramp
    start_value: float = 1.0
    end_value: float = 0.0

    def __post_init__(self) -> None:
        self._t0 = time.monotonic()

    def reset(self) -> None:
        self._t0 = time.monotonic()

    def value(self) -> float:
        """
        Returns TTR in [0,1].
        1.0 = far/low urgency, 0.0 = imminent/high urgency.
        """
        t = time.monotonic() - self._t0

        if self.ramp_seconds <= 0:
            return clamp01(self.end_value)

        if t <= self.ramp_seconds:
            u = t / self.ramp_seconds  # 0..1
            v = self.start_value + u * (self.end_value - self.start_value)
            return clamp01(v)

        # After ramp: hold at end_value indefinitely for now
        return clamp01(self.end_value)
