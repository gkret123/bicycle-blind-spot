"""
Track-level association between radar and vision detections.

Matches radar tracks to camera tracks based on bearing angle proximity.
When a radar track and a camera track correspond to the same physical vehicle,
a FusedTrack is produced that combines the best attributes from each sensor:

  - Range (meters):       from radar  (true range, not bbox proxy)
  - Closing speed (m/s):  from radar  (Doppler + Kalman range-rate)
  - Classification:       from camera (YOLO class: car, motorcycle, bus, truck)
  - Bearing angle:        weighted average (radar for far, camera for near)
  - Confidence:           combined (radar track age + camera YOLO confidence)

Unmatched detections are kept as single-sensor FusedTracks so the fusion
pipeline can still use them (graceful degradation when one sensor drops out).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class SensorSnapshot:
    """Minimal per-frame output from one sensor, used for association."""
    source: str               # "radar" or "vision"
    angle_deg: float          # bearing: negative=left, positive=right
    approaching: bool
    timestamp: float = field(default_factory=time.time)

    # Radar-specific (None when source="vision")
    range_m: Optional[float] = None
    closing_mps: Optional[float] = None
    n_tracks: int = 0
    radar_hits: int = 0       # track hit count (age/confidence proxy)

    # Vision-specific (None when source="radar")
    best_h: Optional[float] = None       # bbox height (px)
    best_dhdt: Optional[float] = None    # bbox growth rate (px/s)
    yolo_conf: Optional[float] = None    # YOLO detection confidence
    status: str = "NO_TARGET"


@dataclass
class FusedTrack:
    """
    A single threat that may have data from radar, vision, or both.

    When both sensors see the same vehicle, the fused track carries
    radar-quality range/speed and camera-quality classification.
    """
    # Which sensors contributed
    has_radar: bool = False
    has_vision: bool = False

    # Fused bearing (weighted average when both present)
    angle_deg: float = 0.0

    # From radar (or None)
    range_m: Optional[float] = None
    closing_mps: Optional[float] = None

    # From vision (or None)
    best_h: Optional[float] = None
    best_dhdt: Optional[float] = None

    # Combined
    approaching: bool = False
    confidence: float = 0.0   # 0-1 composite confidence score
    status: str = "NO_TARGET"

    @property
    def sensor_label(self) -> str:
        if self.has_radar and self.has_vision:
            return "FUSED"
        if self.has_radar:
            return "RADAR"
        return "VISION"


class TrackAssociator:
    """
    Associates radar and vision detections by bearing angle.

    Currently operates on the single best-threat from each sensor
    (matching the existing pipeline design). This can be extended to
    multi-target association when both providers expose full track lists.
    """

    def __init__(
        self,
        angle_gate_deg: float = 12.0,
        max_time_delta_s: float = 0.3,
    ):
        """
        Args:
            angle_gate_deg: Maximum bearing difference (degrees) to consider
                two detections as the same vehicle. 12 degrees accounts for
                radar's coarser angular resolution at range.
            max_time_delta_s: Maximum time gap between radar and vision
                snapshots to attempt association.
        """
        self.angle_gate_deg = angle_gate_deg
        self.max_time_delta_s = max_time_delta_s

    def associate(
        self,
        radar: Optional[SensorSnapshot],
        vision: Optional[SensorSnapshot],
    ) -> List[FusedTrack]:
        """
        Attempt to match radar and vision snapshots.

        Returns a list of FusedTracks:
          - One fused track if both sensors see the same vehicle
          - Two separate tracks if they see different vehicles
          - One single-sensor track if only one sensor has a target
          - Empty list if neither sensor has a target
        """
        r_valid = radar is not None and radar.status != "NO_TARGET"
        v_valid = vision is not None and vision.status != "NO_TARGET"

        if not r_valid and not v_valid:
            return []

        if r_valid and not v_valid:
            return [self._radar_only(radar)]

        if v_valid and not r_valid:
            return [self._vision_only(vision)]

        # Both sensors have targets — check if they match
        angle_diff = abs(radar.angle_deg - vision.angle_deg)
        time_diff = abs(radar.timestamp - vision.timestamp)

        if angle_diff <= self.angle_gate_deg and time_diff <= self.max_time_delta_s:
            return [self._fuse(radar, vision)]
        else:
            # Different vehicles — return both as separate tracks
            return [self._radar_only(radar), self._vision_only(vision)]

    def _radar_only(self, r: SensorSnapshot) -> FusedTrack:
        conf = min(1.0, r.radar_hits / 10.0) if r.radar_hits > 0 else 0.3
        return FusedTrack(
            has_radar=True,
            has_vision=False,
            angle_deg=r.angle_deg,
            range_m=r.range_m,
            closing_mps=r.closing_mps,
            approaching=r.approaching,
            confidence=conf,
            status=r.status,
        )

    def _vision_only(self, v: SensorSnapshot) -> FusedTrack:
        conf = v.yolo_conf if v.yolo_conf is not None else 0.5
        return FusedTrack(
            has_radar=False,
            has_vision=True,
            angle_deg=v.angle_deg,
            best_h=v.best_h,
            best_dhdt=v.best_dhdt,
            approaching=v.approaching,
            confidence=conf,
            status=v.status,
        )

    def _fuse(self, r: SensorSnapshot, v: SensorSnapshot) -> FusedTrack:
        """
        Create a fused track from matched radar + vision detections.

        Bearing: weighted by range — camera is more precise at close range,
        radar is more reliable at long range.
        """
        # Bearing blend: at close range (<10m) favor camera, at far range favor radar
        if r.range_m is not None and r.range_m < 10.0:
            cam_w = 0.7
        else:
            cam_w = 0.3
        fused_angle = cam_w * v.angle_deg + (1.0 - cam_w) * r.angle_deg

        # Confidence: combine radar track age and YOLO confidence
        radar_conf = min(1.0, r.radar_hits / 10.0) if r.radar_hits > 0 else 0.3
        vision_conf = v.yolo_conf if v.yolo_conf is not None else 0.5
        # Both sensors seeing the same target is inherently higher confidence
        combined_conf = min(1.0, 0.5 * radar_conf + 0.5 * vision_conf + 0.2)

        return FusedTrack(
            has_radar=True,
            has_vision=True,
            angle_deg=fused_angle,
            range_m=r.range_m,
            closing_mps=r.closing_mps,
            best_h=v.best_h,
            best_dhdt=v.best_dhdt,
            approaching=r.approaching or v.approaching,
            confidence=combined_conf,
            status=r.status if r.status != "NO_TARGET" else v.status,
        )
