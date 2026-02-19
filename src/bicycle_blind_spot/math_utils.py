"""
Math and scaling utilities.

All numeric helper functions live here.
If you need clamping, scaling, easing, etc.,
this is where it goes.
"""

from .constants import TTR_BYTE_MIN, TTR_BYTE_MAX


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp a value between lo and hi."""
    return lo if x < lo else hi if x > hi else x


def clamp01(x: float) -> float:
    """Clamp a value between 0.0 and 1.0."""
    return clamp(x, 0.0, 1.0)


def ttr_to_byte(ttr: float) -> int:
    """
    Convert TTR scalar [0,1] to a single byte [0,255].

    Convention:
        1.0 = calm (far away)
        0.0 = urgent (vehicle very close)
    """
    ttr = clamp01(ttr)
    value = int(round(ttr * TTR_BYTE_MAX))

    if value < TTR_BYTE_MIN:
        return TTR_BYTE_MIN
    if value > TTR_BYTE_MAX:
        return TTR_BYTE_MAX

    return value


def byte_to_ttr(byte: int) -> float:
    """Convert received byte back to TTR scalar."""
    byte = int(byte)
    if byte < TTR_BYTE_MIN:
        byte = TTR_BYTE_MIN
    if byte > TTR_BYTE_MAX:
        byte = TTR_BYTE_MAX

    return byte / float(TTR_BYTE_MAX)
