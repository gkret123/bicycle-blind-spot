"""
Sensor fusion module for bicycle blind-spot threat detection.

Combines radar and camera pipelines using:
- Track-level association (bearing-angle matching)
- Confidence-weighted urgency fusion (dynamic weights based on range & signal quality)

Provides:
- TrackAssociator: matches radar tracks to vision tracks by bearing angle
- FusionTTRSource: fused TTR source that wraps both VisionTTRSource and RadarTTRSource
"""

from .track_associator import TrackAssociator, FusedTrack
from .ttr_source_fusion import FusionTTRSource, FusionTTRConfig

__all__ = [
    "TrackAssociator",
    "FusedTrack",
    "FusionTTRSource",
    "FusionTTRConfig",
]
