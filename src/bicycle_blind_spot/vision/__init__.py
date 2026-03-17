"""
Dual-camera vision module for bicycle blind-spot threat detection.

Provides:
- CameraProvider: dual Picamera2 + YOLO tracking for Observations
- VisionTTRSource: threat assessment + angle-based left/right TTR split
"""

from .camera_provider import CameraProvider, CameraResult
from .ttr_source_vision import VisionTTRSource, VisionTTRConfig

__all__ = [
    "CameraProvider",
    "CameraResult",
    "VisionTTRSource",
    "VisionTTRConfig",
]
