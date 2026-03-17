"""
Vision-based TTR source.

Wraps CameraProvider and maps its TTC output directly to a TTR scalar [0,1],
then computes angle-based left/right splits.

Interface: .value() -> float  (same as TTRRamp — plugs into TTRStreamer)
Extra:     .left_ttr, .right_ttr, .angle_deg, .status
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from bicycle_blind_spot.vision.camera_provider import CameraProvider, TTC_WARN, TTC_ALERT
from bicycle_blind_spot.utils.math_utils import clamp01


@dataclass
class VisionTTRConfig:
    # TTC thresholds (re-uses the values from camera_provider)
    ttc_calm_s: float = TTC_WARN    # TTC at which TTR = 1.0 (calm)
    ttc_urgent_s: float = 0.5       # TTC at which TTR = 0.0 (maximum urgency)

    # Angle-to-L/R split
    angle_gate_deg: float = 70.0    # clamp angle to ±this before splitting
    angle_boost: float = 0.7        # 0.0 = symmetric, 1.0 = max directional split

    # Camera setup
    cam_left_index: int = 0
    cam_right_index: int = 1
    show: bool = False              # cv2.imshow windows


class VisionTTRSource:
    """
    TTR source backed by dual-camera detection.
    TTC from CameraProvider → TTR scalar → angle-split left/right values.
    """

    def __init__(self, cfg: Optional[VisionTTRConfig] = None):
        self.cfg = cfg or VisionTTRConfig()
        self.provider = CameraProvider(
            cam_left_index=self.cfg.cam_left_index,
            cam_right_index=self.cfg.cam_right_index,
            show=self.cfg.show,
        )
        self._status = "NO_TARGET"
        self._angle_deg = 0.0
        self._left_ttr = 1.0
        self._right_ttr = 1.0
        self._base_ttr = 1.0

    def value(self) -> float:
        """
        Step cameras, compute TTR. Called by TTRStreamer at the configured Hz.
        Returns base TTR [0,1] where 1=calm, 0=urgent.
        """
        result = self.provider.read()
        self._status = result.status

        # Map TTC to TTR: linear between calm and urgent thresholds
        if result.status == "NO_TARGET":
            base_ttr = 1.0
        else:
            span = self.cfg.ttc_calm_s - self.cfg.ttc_urgent_s
            base_ttr = clamp01((result.ttc_s - self.cfg.ttc_urgent_s) / span)

        self._base_ttr = base_ttr
        self._angle_deg = result.angle_deg
        self._compute_splits(base_ttr, result.angle_deg)
        return base_ttr

    def _compute_splits(self, base_ttr: float, angle_deg: float):
        """
        Split base TTR between left and right devices by angle.

        Convention: negative angle = vehicle left, positive = vehicle right.

        The far side is eased toward calm (TTR raised toward 1.0).
        The near side keeps the base TTR unchanged.

        With angle_boost=0.7 and a vehicle at -45°:
          split_factor = -0.64
          left_ttr  = base_ttr  (near side, no change)
          right_ttr = base_ttr + 0.64 * (1-base_ttr) * 0.7  (far side, eased up)
        """
        angle_clamped = max(-self.cfg.angle_gate_deg,
                            min(self.cfg.angle_gate_deg, angle_deg))
        split_factor = angle_clamped / self.cfg.angle_gate_deg  # [-1, +1]

        # Far side gets eased (raised toward 1.0); near side unchanged
        left_ease  = max(0.0,  split_factor) * (1.0 - base_ttr) * self.cfg.angle_boost
        right_ease = max(0.0, -split_factor) * (1.0 - base_ttr) * self.cfg.angle_boost

        self._left_ttr  = clamp01(base_ttr + left_ease)
        self._right_ttr = clamp01(base_ttr + right_ease)

    @property
    def status(self) -> str:
        return self._status

    @property
    def angle_deg(self) -> float:
        return self._angle_deg

    @property
    def left_ttr(self) -> float:
        return self._left_ttr

    @property
    def right_ttr(self) -> float:
        return self._right_ttr

    def stop(self):
        self.provider.stop()
