"""
Radar-based TTR source.

Wraps RadarProvider and computes a TTR (Time-To-React) scalar from two
independent urgency components:

  1. **Proximity** — how close the vehicle is (range in meters).
     A shorter range means the vehicle is nearby.
  2. **Closing speed** — how fast the vehicle is approaching (m/s).
     A high closing speed means rapid approach. This uses a fused estimate:
     signed radar radial velocity plus geometric range-rate from the tracker.
     
IMPORTANT: Urgency is ONLY produced when the vehicle is actively approaching
(closing_mps > min_approach_mps).  A vehicle at matched speed, stationary,
or receding produces TTR = 1.0 (calm, no vibration).

TTR = 1.0 - clamp01(w_proximity * proximity + w_closing * closing)

The angle of the best threat determines which side vibrates:
  angle < -both_zone_deg  →  left side only
  |angle| ≤ both_zone_deg →  both sides
  angle > +both_zone_deg  →  right side only

All TTR outputs are EMA-smoothed for smooth vibration transitions.

The radar loop runs in a dedicated background thread so that .value()
returns instantly and the BLE stream can maintain its configured Hz.

Interface: .value() -> float  (same as VisionTTRSource — plugs into TTRStreamer)
Extra:     .left_ttr, .right_ttr, .angle_deg, .status, .approaching
"""

from __future__ import annotations

import threading
import traceback
from dataclasses import dataclass
from typing import Optional

from bicycle_blind_spot.radar.radar_provider import RadarProvider, RadarConfig
from bicycle_blind_spot.utils.math_utils import clamp01


@dataclass
class RadarTTRConfig:
    # ---- Proximity component (range in meters) ----
    # range_far: beyond this, proximity urgency = 0 (vehicle too far to matter)
    # range_close: within this, proximity urgency = 1 (maximum danger)
    range_far: float = 80.0       # meters
    range_close: float = 3.0      # meters
    w_proximity: float = 0.4      # weight in combined urgency

    # ---- Closing-speed component (m/s) ----
    # closing_max: speed at which closing urgency is maximal
    closing_max: float = 15.0     # m/s (~54 km/h relative approach speed)
    w_closing: float = 0.6        # weight in combined urgency

    # ---- Approach gate ----
    # Vehicle must be approaching faster than this (m/s) for ANY vibration.
    # If closing_mps <= this threshold, TTR = 1.0.
    min_approach_mps: float = 0.5  # m/s — below this, no vibration

    # ---- Output smoothing ----
    # EMA alpha applied to left/right TTR each radar frame.
    ttr_smooth_alpha: float = 0.15

    # ---- Angle-based side selection ----
    # Within ±both_zone_deg of straight-back, both sides vibrate.
    both_zone_deg: float = 10.0

    # ---- Radar hardware config ----
    cfg_port: str = "/dev/ttyUSB0"
    data_port: str = "/dev/ttyUSB1"
    range_preset: str = "long"
    approaching_sign: int = 1     # +1 or -1, calibrate on first test

    # ---- Visualization ----
    show: bool = False            # enable live top-down matplotlib plot
    verbose: bool = False         # print per-frame radar summaries

class RadarTTRSource:
    """
    TTR source backed by TI mmWave radar.

    Radar capture + tracking run in a background thread (started
    explicitly via .start()).  .value() and the property accessors return
    the latest cached result without blocking, so TTRStreamer can maintain
    its BLE write cadence.
    """

    def __init__(self, cfg: Optional[RadarTTRConfig] = None):
        self.cfg = cfg or RadarTTRConfig()

        radar_cfg = RadarConfig(
            cfg_port=self.cfg.cfg_port,
            data_port=self.cfg.data_port,
            range_preset=self.cfg.range_preset,
            approaching_sign=self.cfg.approaching_sign,
            show=self.cfg.show,
            verobose=self.cfg.verbose,
        )
        self.provider = RadarProvider(cfg=radar_cfg)

        # Shared state (guarded by _lock)
        self._lock = threading.Lock()
        self._status = "NO_TARGET"
        self._angle_deg = 0.0
        self._left_ttr = 1.0
        self._right_ttr = 1.0
        self._base_ttr = 1.0
        self._approaching = False

        # EMA-smoothed outputs (updated in radar thread only)
        self._smooth_left = 1.0
        self._smooth_right = 1.0

        # Background radar loop (call .start() to begin)
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        """Configure sensor and begin the background radar loop."""
        if self._thread is not None:
            return
        self.provider.configure()
        self.provider.start_data()
        self._running = True
        self._thread = threading.Thread(target=self._radar_loop, daemon=True)
        self._thread.start()

    def _radar_loop(self):
        """Continuously read from radar and update cached TTR state."""
        alpha = self.cfg.ttr_smooth_alpha

        while self._running:
            try:
                result = self.provider.read()

                # Gate: only produce urgency when vehicle is actively approaching.
                approaching = (
                    result.status != "NO_TARGET"
                    and result.closing_mps > self.cfg.min_approach_mps
                )

                if approaching:
                    base_ttr = self._compute_ttr(result.range_m, result.closing_mps)
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

    def _compute_ttr(self, range_m: float, closing_mps: float) -> float:
        """
        Two-factor urgency model.

        Proximity: closer → more urgent (inverted range).
        Closing speed: faster approach → more urgent.

        Only called when vehicle IS approaching (closing_mps > min_approach_mps).

        Returns TTR in [0, 1] where 1.0 = calm, 0.0 = maximum urgency.
        """
        cfg = self.cfg

        # Proximity: normalize range to [0, 1] urgency (inverted — closer = higher)
        r_span = cfg.range_far - cfg.range_close
        proximity = clamp01((cfg.range_far - range_m) / r_span) if r_span > 0 else 0.0

        # Closing speed: only positive (approaching) counts
        closing = clamp01(max(0.0, closing_mps) / cfg.closing_max) if cfg.closing_max > 0 else 0.0

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
        """True when the best threat is actively approaching."""
        with self._lock:
            return self._approaching

    def step_viz(self):
        """Forward to RadarProvider.step_viz() — call from main thread when show=True."""
        self.provider.step_viz()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.provider.stop()
