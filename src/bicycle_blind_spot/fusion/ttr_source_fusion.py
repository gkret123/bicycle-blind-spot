"""
Fused sensor TTR source — combines radar and camera pipelines.

Implements two complementary fusion strategies:

  1. **Track-level fusion** (TrackAssociator):
     When both sensors detect the same vehicle (bearing match within gate),
     the fused track carries radar-quality range/speed with camera-quality
     classification. Unmatched detections degrade gracefully to single-sensor.

  2. **Confidence-weighted urgency fusion**:
     Each sensor computes an independent urgency score. The final urgency is
     a weighted blend where the weights adapt dynamically based on signal
     quality and range:

       - Far range (>15 m):  radar dominates  (true range + Doppler)
       - Near range (<5 m):  camera dominates (large bbox, radar clutter)
       - Mid range (5-15 m): both contribute equally
       - Sensor dropout:     available sensor takes full weight

     urgency_fused = w_r * urgency_radar + w_v * urgency_vision

TTR = 1.0 - clamp01(urgency_fused)

The angle of the most urgent fused track determines which side vibrates,
using the same zone-based L/R split as the single-sensor pipelines.

All outputs are EMA-smoothed. The fusion loop runs in a dedicated background
thread so that .value() returns instantly for the BLE streamer.

Interface: identical to VisionTTRSource / RadarTTRSource — drop-in replacement.
"""

from __future__ import annotations

import threading
import time
import traceback
from dataclasses import dataclass
from typing import Optional

import serial

from bicycle_blind_spot.vision.camera_provider import CameraProvider, CameraResult, FRAME_H
from bicycle_blind_spot.vision.ttr_source_vision import VisionTTRConfig
from bicycle_blind_spot.radar.radar_provider import RadarProvider, RadarConfig, RadarResult
from bicycle_blind_spot.radar.ttr_source_radar import RadarTTRConfig
from bicycle_blind_spot.fusion.track_associator import (
    TrackAssociator, SensorSnapshot, FusedTrack,
)
from bicycle_blind_spot.utils.math_utils import clamp01


@dataclass
class FusionTTRConfig:
    """Configuration for the fused sensor TTR source."""

    # ---- Vision sub-config ----
    cam_left_index: int = 0
    cam_right_index: int = 1
    cam_show: bool = False
    cam_l_offset_deg: float = -35.0
    cam_r_offset_deg: float = +35.0

    # Vision urgency parameters
    h_far: float = 0.05
    h_close: float = 0.45
    dhdt_max: float = 40.0
    min_approach_dhdt: float = 0.5

    # ---- Radar sub-config ----
    cfg_port: str = "auto"
    data_port: str = "auto"
    range_preset: str = "long"
    approaching_sign: int = 1
    radar_show: bool = False
    verbose_radar: bool = False

    # Radar urgency parameters
    range_far: float = 80.0
    range_close: float = 3.0
    closing_max: float = 15.0
    min_approach_mps: float = 0.5

    # ---- Fusion parameters ----
    # Zone boundaries for dynamic weight crossfade (meters)
    near_range_m: float = 5.0     # below this, camera dominates
    far_range_m: float = 15.0     # above this, radar dominates

    # Base sensor weights (before dynamic adjustment)
    # These are the weights at mid-range where both contribute equally.
    w_radar_base: float = 0.5
    w_vision_base: float = 0.5

    # Track association
    angle_gate_deg: float = 12.0
    max_time_delta_s: float = 0.3
    fusion_strategy: str = "confidence_weighted"  # confidence_weighted | track_level

    # ---- Shared output config ----
    ttr_smooth_alpha: float = 0.15
    both_zone_deg: float = 10.0


class FusionTTRSource:
    """
    Fused TTR source backed by both camera and radar pipelines.

    Both sensor providers run in their own background threads.
    The fusion loop runs in a third thread that reads the latest
    result from each sensor, associates tracks, computes fused
    urgency, and updates the shared TTR state.

    .value(), .left_ttr, .right_ttr etc. are non-blocking and
    thread-safe — identical interface to VisionTTRSource / RadarTTRSource.
    """

    def __init__(self, cfg: Optional[FusionTTRConfig] = None):
        self.cfg = cfg or FusionTTRConfig()

        # Build sub-providers
        self._vision_provider = CameraProvider(
            cam_left_index=self.cfg.cam_left_index,
            cam_right_index=self.cfg.cam_right_index,
            show=self.cfg.cam_show,
            cam_l_offset_deg=self.cfg.cam_l_offset_deg,
            cam_r_offset_deg=self.cfg.cam_r_offset_deg,
        )

        radar_cfg = RadarConfig(
            cfg_port=self.cfg.cfg_port,
            data_port=self.cfg.data_port,
            range_preset=self.cfg.range_preset,
            approaching_sign=self.cfg.approaching_sign,
            show=self.cfg.radar_show,
            verbose=self.cfg.verbose_radar,
        )
        self._radar_provider = RadarProvider(cfg=radar_cfg)

        self._associator = TrackAssociator(
            angle_gate_deg=self.cfg.angle_gate_deg,
            max_time_delta_s=self.cfg.max_time_delta_s,
        )

        # Shared state (guarded by _lock)
        self._lock = threading.Lock()
        self._status = "NO_TARGET"
        self._angle_deg = 0.0
        self._left_ttr = 1.0
        self._right_ttr = 1.0
        self._base_ttr = 1.0
        self._approaching = False
        self._fused_label = ""   # "FUSED", "RADAR", or "VISION"

        # EMA-smoothed outputs
        self._smooth_left = 1.0
        self._smooth_right = 1.0

        # Latest sensor results (written by sensor threads, read by fusion thread)
        self._vision_lock = threading.Lock()
        self._latest_vision: Optional[CameraResult] = None
        self._vision_ts: float = 0.0

        self._radar_lock = threading.Lock()
        self._latest_radar: Optional[RadarResult] = None
        self._radar_ts: float = 0.0

        # Threads
        self._running = False
        self._vision_thread: Optional[threading.Thread] = None
        self._radar_thread: Optional[threading.Thread] = None
        self._fusion_thread: Optional[threading.Thread] = None

    def start(self):
        """Configure radar and start all background threads."""
        if self._fusion_thread is not None:
            return

        # Configure and start radar hardware
        self._radar_provider.configure()
        self._radar_provider.start_data()

        self._running = True

        self._vision_thread = threading.Thread(
            target=self._vision_loop, daemon=True, name="fusion-vision",
        )
        self._radar_thread = threading.Thread(
            target=self._radar_loop, daemon=True, name="fusion-radar",
        )
        self._fusion_thread = threading.Thread(
            target=self._fusion_loop, daemon=True, name="fusion-main",
        )

        self._vision_thread.start()
        self._radar_thread.start()
        self._fusion_thread.start()

    # ------------------------------------------------------------------ #
    # Sensor threads — each reads its provider and stashes the result
    # ------------------------------------------------------------------ #

    def _vision_loop(self):
        while self._running:
            try:
                result = self._vision_provider.read()
                with self._vision_lock:
                    self._latest_vision = result
                    self._vision_ts = time.time()
            except Exception:
                traceback.print_exc()

    def _radar_loop(self):
        while self._running:
            try:
                result = self._radar_provider.read()
                with self._radar_lock:
                    self._latest_radar = result
                    self._radar_ts = time.time()
            except serial.SerialException:
                print("[radar] Serial disconnect (EMI?) — reconnecting…")
                try:
                    self._radar_provider.reconnect_data()
                except Exception:
                    traceback.print_exc()
                    time.sleep(2.0)
            except Exception:
                traceback.print_exc()

    # ------------------------------------------------------------------ #
    # Fusion thread — reads both latest results, associates, computes TTR
    # ------------------------------------------------------------------ #

    def _fusion_loop(self):
        alpha = self.cfg.ttr_smooth_alpha

        while self._running:
            try:
                now = time.time()

                # Grab latest from each sensor
                with self._vision_lock:
                    v_result = self._latest_vision
                    v_ts = self._vision_ts
                with self._radar_lock:
                    r_result = self._latest_radar
                    r_ts = self._radar_ts

                if self.cfg.fusion_strategy == "track_level":
                    # Build snapshots for the associator
                    v_snap = self._vision_snapshot(v_result, v_ts, now)
                    r_snap = self._radar_snapshot(r_result, r_ts, now)

                    # Track-level association
                    fused_tracks = self._associator.associate(r_snap, v_snap)

                    if not fused_tracks:
                        base_ttr = 1.0
                        angle = 0.0
                        approaching = False
                        status = "NO_TARGET"
                        fused_label = ""
                    else:
                        # Pick the most urgent fused track
                        best = self._select_best_fused(fused_tracks, v_result, r_result)
                        base_ttr = self._compute_fused_ttr(best, v_result, r_result)
                        angle = best.angle_deg
                        approaching = best.approaching
                        status = best.status
                        fused_label = best.sensor_label

                        # Gate: no vibration unless approaching
                        if not approaching:
                            base_ttr = 1.0
                else:
                    base_ttr, angle, approaching, status, fused_label = (
                        self._confidence_weighted_fusion(v_result, r_result)
                    )

                # Zone-based angle split
                raw_left, raw_right = self._compute_side_split(base_ttr, angle)

                # EMA smooth
                self._smooth_left += alpha * (raw_left - self._smooth_left)
                self._smooth_right += alpha * (raw_right - self._smooth_right)

                with self._lock:
                    self._base_ttr = base_ttr
                    self._status = status
                    self._angle_deg = angle
                    self._left_ttr = self._smooth_left
                    self._right_ttr = self._smooth_right
                    self._approaching = approaching
                    self._fused_label = fused_label

                # Run at ~50 Hz to keep up with both sensors
                time.sleep(0.02)

            except Exception:
                traceback.print_exc()
                time.sleep(0.05)

    # ------------------------------------------------------------------ #
    # Snapshot builders
    # ------------------------------------------------------------------ #

    def _vision_snapshot(
        self, result: Optional[CameraResult], ts: float, now: float,
    ) -> Optional[SensorSnapshot]:
        if result is None or (now - ts) > 1.0:
            return None
        return SensorSnapshot(
            source="vision",
            angle_deg=result.angle_deg,
            approaching=result.approaching,
            timestamp=ts,
            best_h=result.best_h,
            best_dhdt=result.best_dhdt,
            status=result.status,
        )

    def _radar_snapshot(
        self, result: Optional[RadarResult], ts: float, now: float,
    ) -> Optional[SensorSnapshot]:
        if result is None or (now - ts) > 1.0:
            return None
        return SensorSnapshot(
            source="radar",
            angle_deg=result.angle_deg,
            approaching=result.approaching,
            timestamp=ts,
            range_m=result.range_m,
            closing_mps=result.closing_mps,
            n_tracks=result.n_tracks,
            radar_hits=result.n_tracks,  # proxy — confirmed tracks count
            status=result.status,
        )

    # ------------------------------------------------------------------ #
    # Best-track selection from fused tracks
    # ------------------------------------------------------------------ #

    def _select_best_fused(
        self,
        tracks: list[FusedTrack],
        v_result: Optional[CameraResult],
        r_result: Optional[RadarResult],
    ) -> FusedTrack:
        """Pick the most urgent fused track."""
        best = tracks[0]
        best_urgency = self._compute_fused_ttr(best, v_result, r_result)

        for t in tracks[1:]:
            u = self._compute_fused_ttr(t, v_result, r_result)
            if u < best_urgency:  # lower TTR = more urgent
                best_urgency = u
                best = t
        return best

    # ------------------------------------------------------------------ #
    # Confidence-weighted urgency computation
    # ------------------------------------------------------------------ #

    def _compute_fused_ttr(
        self,
        track: FusedTrack,
        v_result: Optional[CameraResult],
        r_result: Optional[RadarResult],
    ) -> float:
        """
        Compute fused TTR using confidence-weighted urgency blending.

        Dynamic weights based on range:
          - Far (>far_range_m):  radar 0.85, vision 0.15
          - Mid (near..far):    linear crossfade
          - Near (<near_range_m): radar 0.15, vision 0.85
          - Single sensor:      that sensor gets weight 1.0
        """
        cfg = self.cfg

        have_radar_urgency = track.has_radar and track.approaching
        have_vision_urgency = track.has_vision and track.approaching

        # Compute individual urgencies
        radar_urgency = 0.0
        if have_radar_urgency and track.range_m is not None and track.closing_mps is not None:
            radar_urgency = self._radar_urgency(track.range_m, track.closing_mps)

        vision_urgency = 0.0
        if have_vision_urgency and track.best_h is not None and track.best_dhdt is not None:
            vision_urgency = self._vision_urgency(track.best_h, track.best_dhdt)

        # Single-sensor fallback
        if have_radar_urgency and not have_vision_urgency:
            return 1.0 - clamp01(radar_urgency)
        if have_vision_urgency and not have_radar_urgency:
            return 1.0 - clamp01(vision_urgency)
        if not have_radar_urgency and not have_vision_urgency:
            return 1.0

        # Both sensors contributing — compute dynamic weights
        w_r, w_v = self._dynamic_weights(track.range_m, track.confidence)

        fused_urgency = w_r * radar_urgency + w_v * vision_urgency
        return 1.0 - clamp01(fused_urgency)

    def _dynamic_weights(
        self,
        range_m: Optional[float],
        confidence: float,
    ) -> tuple[float, float]:
        """
        Compute dynamic radar/vision weights based on range.

        Returns (w_radar, w_vision) that sum to 1.0.
        """
        cfg = self.cfg

        if range_m is None:
            return cfg.w_radar_base, cfg.w_vision_base

        if range_m >= cfg.far_range_m:
            # Far: radar dominates
            w_r = 0.85
        elif range_m <= cfg.near_range_m:
            # Near: camera dominates
            w_r = 0.15
        else:
            # Mid: linear crossfade from 0.15 (near) to 0.85 (far)
            t = (range_m - cfg.near_range_m) / (cfg.far_range_m - cfg.near_range_m)
            w_r = 0.15 + t * 0.70

        w_v = 1.0 - w_r
        return w_r, w_v

    def _confidence_weighted_fusion(
        self,
        v_result: Optional[CameraResult],
        r_result: Optional[RadarResult],
    ) -> tuple[float, float, bool, str, str]:
        """
        Fusion without hard association.

        This is the recommended starting point: blend per-sensor urgency
        using adaptive weights and confidence scores.
        """
        have_v = (
            v_result is not None
            and v_result.status != "NO_TARGET"
            and v_result.approaching
            and v_result.best_h > 0.0
        )
        have_r = (
            r_result is not None
            and r_result.status != "NO_TARGET"
            and r_result.approaching
            and r_result.range_m > 0.0
        )

        if not have_v and not have_r:
            return 1.0, 0.0, False, "NO_TARGET", ""

        v_urg = self._vision_urgency(v_result.best_h, v_result.best_dhdt) if have_v else 0.0
        r_urg = self._radar_urgency(r_result.range_m, r_result.closing_mps) if have_r else 0.0

        if have_v and not have_r:
            return 1.0 - clamp01(v_urg), v_result.angle_deg, True, v_result.status, "VISION"
        if have_r and not have_v:
            return 1.0 - clamp01(r_urg), r_result.angle_deg, True, r_result.status, "RADAR"

        # Both sensors are active: adaptive weight by range + confidence
        w_r, w_v = self._dynamic_weights(r_result.range_m, 0.5)
        radar_conf = clamp01(min(1.0, r_result.n_tracks / 8.0))
        vision_conf = clamp01(0.4 + min(0.6, v_result.best_dhdt / max(self.cfg.dhdt_max, 1.0)))
        w_r *= radar_conf
        w_v *= vision_conf
        denom = max(1e-6, w_r + w_v)
        w_r /= denom
        w_v /= denom

        fused_urg = clamp01(w_r * r_urg + w_v * v_urg)
        angle = (w_r * r_result.angle_deg + w_v * v_result.angle_deg)

        status = "ALERT" if ("ALERT" in (v_result.status, r_result.status)) else "WARN"
        return 1.0 - fused_urg, angle, True, status, "FUSED-CW"

    def _radar_urgency(self, range_m: float, closing_mps: float) -> float:
        """Radar urgency: proximity (inverted range) + closing speed."""
        cfg = self.cfg
        r_span = cfg.range_far - cfg.range_close
        proximity = clamp01((cfg.range_far - range_m) / r_span) if r_span > 0 else 0.0
        closing = clamp01(max(0.0, closing_mps) / cfg.closing_max) if cfg.closing_max > 0 else 0.0
        # Use the same 0.4/0.6 split as standalone radar
        return 0.4 * proximity + 0.6 * closing

    def _vision_urgency(self, best_h: float, best_dhdt: float) -> float:
        """Vision urgency: proximity (bbox size) + closing speed (bbox growth)."""
        cfg = self.cfg
        h_ratio = best_h / FRAME_H
        h_span = cfg.h_close - cfg.h_far
        proximity = clamp01((h_ratio - cfg.h_far) / h_span) if h_span > 0 else 0.0
        closing = clamp01(max(0.0, best_dhdt) / cfg.dhdt_max) if cfg.dhdt_max > 0 else 0.0
        # Use the same 0.4/0.6 split as standalone vision
        return 0.4 * proximity + 0.6 * closing

    # ------------------------------------------------------------------ #
    # Side split (same as single-sensor pipelines)
    # ------------------------------------------------------------------ #

    def _compute_side_split(
        self, base_ttr: float, angle_deg: float,
    ) -> tuple[float, float]:
        zone = self.cfg.both_zone_deg
        if angle_deg <= -zone:
            return base_ttr, 1.0
        elif angle_deg >= zone:
            return 1.0, base_ttr
        else:
            return base_ttr, base_ttr

    # ------------------------------------------------------------------ #
    # Public interface (matches VisionTTRSource / RadarTTRSource)
    # ------------------------------------------------------------------ #

    def value(self) -> float:
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
        with self._lock:
            return self._approaching

    @property
    def fused_label(self) -> str:
        """Which sensors contributed to the current output: FUSED, RADAR, or VISION."""
        with self._lock:
            return self._fused_label

    def step_viz(self):
        """Drive all sensor visualizations — call from main thread when any show flag is set."""
        self._radar_provider.step_viz()
        self._vision_provider.step_viz()

    def stop(self):
        self._running = False
        for t in (self._vision_thread, self._radar_thread, self._fusion_thread):
            if t is not None:
                t.join(timeout=2.0)
        self._vision_provider.stop()
        self._radar_provider.stop()
