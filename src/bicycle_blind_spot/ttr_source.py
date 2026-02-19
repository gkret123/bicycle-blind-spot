"""
TTR Source Module.

Right now this produces a simple ramp from 1.0 to 0.0.

TODO: Replace the internals of this class with real TTR calculation from range and relative velocity
"""

import time
from dataclasses import dataclass
from .math_utils import clamp01


@dataclass
class TTRRamp:
    """
    Generates a time-based TTR ramp.

    Example:
        ramp_seconds = 45
        produces a 45 second linear ramp
        from 1.0 -> 0.0
    """

    ramp_seconds: float = 45.0
    hold_seconds: float = 10.0
    start_value: float = 1.0
    end_value: float = 0.0

    def __post_init__(self):
        self._t0 = time.monotonic()

    def reset(self):
        """Restart the ramp."""
        self._t0 = time.monotonic()

    def value(self) -> float:
        """
        Returns current TTR scalar in [0,1].
        """
        elapsed = time.monotonic() - self._t0

        if elapsed <= self.ramp_seconds:
            progress = elapsed / self.ramp_seconds
            val = self.start_value + progress * (self.end_value - self.start_value)
            return clamp01(val)

        # After ramp completes, stay at end_value
        return clamp01(self.end_value)
