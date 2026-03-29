"""
Vision-based TTR source.

Wraps CameraProvider and computes a TTR (Time-To-React) scalar from two
independent urgency components:

  1. **Proximity** — how close the vehicle is (bbox height / frame height).
     A large bbox means the vehicle is nearby.
  2. **Closing speed** — how fast the vehicle is approaching (bbox growth rate).
     A high positive dh/dt means rapid approach.

IMPORTANT: Urgency is ONLY produced when the vehicle is actively approaching
(bbox growth rate > min_approach_dhdt).  A vehicle at matched speed, stationary,
or receding produces TTR = 1.0 (calm, no vibration).

TTR = 1.0 - clamp01(w_proximity * proximity + w_closing * closing)

The angle of the best threat determines which side vibrates:
  angle < -both_zone_deg  →  left side only
  |angle| ≤ both_zone_deg →  both sides
  angle > +both_zone_deg  →  right side only

All TTR outputs are EMA-smoothed for smooth vibration transitions.

The camera loop runs in a dedicated background thread so that .value()
returns instantly and the BLE stream can maintain its configured Hz.

Interface: .value() -> float  (same as TTRRamp — plugs into TTRStreamer)
Extra:     .left_ttr, .right_ttr, .angle_deg, .status, .approaching
"""

from __future__ import annotations

import threading
import traceback
from dataclasses import dataclass
from typing import Optional

from bicycle_blind_spot.vision.camera_provider import (
    CameraProvider, FRAME_H,
    CAM_L_OFFSET_DEG, CAM_R_OFFSET_DEG,
)
from bicycle_blind_spot.utils.math_utils import clamp01


@dataclass
class VisionTTRConfig:
    # ---- Proximity component (bbox height / frame height) ----
    # h_ratio below h_far produces zero proximity urgency.
    # h_ratio above h_close produces maximum proximity urgency.
    h_far: float = 0.05       # ~14px on 288px frame (far, small bbox)
    h_close: float = 0.45     # ~130px on 288px frame (very close, large bbox)
    w_proximity: float = 0.4  # weight in combined urgency

    # ---- Closing-speed component (bbox growth rate, px/s) ----
    # dhdt_max is the growth rate that produces maximum closing urgency.
    dhdt_max: float = 40.0    # pixels/sec
    w_closing: float = 0.6    # weight in combined urgency

    # ---- Approach gate ----
    # Vehicle must be approaching faster than this (in px/s of bbox growth)
    # for ANY vibration to occur.  If dhdt <= this threshold, TTR = 1.0.
    # Set to 0.0 to vibrate for any positive growth.
    # Set higher (e.g. 1.0-2.0) to ignore very slow approaches.
    min_approach_dhdt: float = 0.5  # px/s — below this, no vibration

    # ---- Output smoothing ----
    # EMA alpha applied to left/right TTR each vision frame.
    # Lower = smoother but slower to respond. Higher = snappier but jumpier.
    # 0.15 gives ~6-frame settling at vision FPS; 0.3 gives ~3-frame settling.
    ttr_smooth_alpha: float = 0.15

    # ---- Angle-based side selection ----
    # Within ±both_zone_deg of straight-back, both sides vibrate.
    # Outside that zone, only the near side vibrates.
    both_zone_deg: float = 10.0

    # ---- Camera setup ----
    cam_left_index: int = 0
    cam_right_index: int = 1
    show: bool = False

    # Camera mount angles (degrees from straight-back)
    cam_l_offset_deg: float = CAM_L_OFFSET_DEG   # negative = left
    cam_r_offset_deg: float = CAM_R_OFFSET_DEG   # positive = right


class VisionTTRSource:
    """
    TTR source backed by dual-camera detection.

    Camera capture + YOLO inference run in a background thread (started
    explicitly via .start()).  .value() and the property accessors return
    the latest cached result without blocking, so TTRStreamer can maintain
    its BLE write cadence.
    """

    def __init__(self, cfg: Optional[VisionTTRConfig] = None):
        self.cfg = cfg or VisionTTRConfig()
        self.provider = CameraProvider(
            cam_left_index=self.cfg.cam_left_index,
            cam_right_index=self.cfg.cam_right_index,
            show=self.cfg.show,
            cam_l_offset_deg=self.cfg.cam_l_offset_deg,
            cam_r_offset_deg=self.cfg.cam_r_offset_deg,
        )

        # Shared state (guarded by _lock)
        self._lock = threading.Lock()
        self._status = "NO_TARGET"
        self._angle_deg = 0.0
        self._left_ttr = 1.0
        self._right_ttr = 1.0
        self._base_ttr = 1.0
        self._approaching = False

        # EMA-smoothed outputs (updated in vision thread only)
        self._smooth_left = 1.0
        self._smooth_right = 1.0

        # Background vision loop (call .start() to begin)
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        """Begin the background vision loop. Call after BLE devices are connected."""
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._vision_loop, daemon=True)
        self._thread.start()

    def _vision_loop(self):
        """Continuously read from cameras and update cached TTR state."""
        alpha = self.cfg.ttr_smooth_alpha

        while self._running:
            try:
                result = self.provider.read()

                # Gate: only produce urgency when vehicle is actively approaching.
                # Matched speed, stationary, or receding → calm (no vibration).
                approaching = (
                    result.status != "NO_TARGET"
                    and result.best_dhdt > self.cfg.min_approach_dhdt
                )

                if approaching:
                    base_ttr = self._compute_ttr(result.best_h, result.best_dhdt)
                else:
                    base_ttr = 1.0

                # Zone-based angle split (raw, before smoothing)
                raw_left, raw_right = self._compute_side_split(
                    base_ttr, result.angle_deg,
                )

                # EMA smooth for gradual vibration transitions
                self._smooth_left += alpha * (raw_left - self._smooth_left)
                self._smooth_right += alpha * (raw_right - self._smooth_right)

                with self._lock:
                    self._base_ttr = base_ttr
                    self._status = result.status
                    self._angle_deg = result.angle_deg
                    self._left_ttr = self._smooth_left
                    self._right_ttr = self._smooth_right
                    self._approaching = approaching
            except Exception:
                traceback.print_exc()

    def _compute_ttr(self, best_h: float, best_dhdt: float) -> float:
        """
        Two-factor urgency model.

        Proximity: bigger bbox → vehicle is closer → more urgent.
        Closing speed: faster bbox growth → vehicle approaching faster → more urgent.

        Only called when vehicle IS approaching (dhdt > min_approach_dhdt).

        Returns TTR in [0, 1] where 1.0 = calm, 0.0 = maximum urgency.
        """
        cfg = self.cfg

        # Proximity: normalize bbox height to [0, 1] urgency
        h_ratio = best_h / FRAME_H
        h_span = cfg.h_close - cfg.h_far
        proximity = clamp01((h_ratio - cfg.h_far) / h_span) if h_span > 0 else 0.0

        # Closing speed: only positive growth counts (approaching)
        closing = clamp01(max(0.0, best_dhdt) / cfg.dhdt_max) if cfg.dhdt_max > 0 else 0.0

        # Weighted sum
        urgency = cfg.w_proximity * proximity + cfg.w_closing * closing

        return 1.0 - clamp01(urgency)

    def _compute_side_split(
        self, base_ttr: float, angle_deg: float,
    ) -> tuple[float, float]:
        """
        Zone-based side selection.

        angle ≤ -both_zone_deg  →  left vibrates, right calm
        |angle| < both_zone_deg →  both vibrate
        angle ≥ +both_zone_deg  →  right vibrates, left calm
        """
        zone = self.cfg.both_zone_deg

        if angle_deg <= -zone:
            return base_ttr, 1.0       # left only
        elif angle_deg >= zone:
            return 1.0, base_ttr       # right only
        else:
            return base_ttr, base_ttr  # both sides

    def value(self) -> float:
        """
        Return latest base TTR [0,1]. Non-blocking.
        Called by TTRStreamer at the configured Hz.
        """
        with self._lock:
            return self._base_ttr

    @property
    def status(self) -> str:
        with self._lock:
            return self._status

    @property
    def angle_deg(self) -> float:
        with self._lock:
            return self._angle_deg

    @property
    def left_ttr(self) -> float:
        with self._lock:
            return self._left_ttr

    @property
    def right_ttr(self) -> float:
        with self._lock:
            return self._right_ttr

    @property
    def approaching(self) -> bool:
        """True when the best threat's bbox is growing faster than min_approach_dhdt."""
        with self._lock:
            return self._approaching

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.provider.stop()

    def step_viz(self):
        """Flush visualization windows from the main thread."""
        self.provider.step_viz()
