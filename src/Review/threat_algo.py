"""
threat_algo.py

Sensor-agnostic rear-threat algorithm that supports "drop-in" radar and/or camera
providers. The core threat logic consumes a unified list of Observations with
fields that ANY sensor can provide (even if approximate).

- Radar provider can output true range/closing speed/angle.
- Camera provider can output approximations (range_proxy / closing_proxy / bearing).
- Fusion provider can combine both and still return the same Observation objects.

You can plug in providers without changing the threat model.

Run mode examples:
  - Radar-only:       ThreatEngine([RadarProvider(...)])
  - Camera-only:      ThreatEngine([CameraProvider(...)])
  - Radar+Camera:     ThreatEngine([RadarProvider(...), CameraProvider(...)])
  - Fused provider:   ThreatEngine([FusionProvider(...)])
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional, Protocol, Dict, Tuple
import time
import math


# ----------------------------- Data Models -----------------------------

@dataclass
class Observation:
    """
    Unified object observation from any sensor (radar/camera/fusion).

    Units:
      - range_m: meters (None if unknown)
      - closing_mps: positive means closing (approaching) in m/s (None if unknown)
      - angle_deg: bearing relative to straight-back; left negative, right positive (None if unknown)
    """
    source: str                  # "radar" | "camera" | "fusion" | ...
    obj_id: Optional[int]        # stable track ID if available (radar often has this)
    timestamp: float

    # Physics-ish fields
    range_m: Optional[float] = None
    closing_mps: Optional[float] = None
    angle_deg: Optional[float] = None

    # Vision-ish fields (optional)
    cls: Optional[str] = None
    confidence: Optional[float] = None
    bbox_xyxy: Optional[Tuple[int, int, int, int]] = None  # x1,y1,x2,y2

    # Generic "quality" hints (optional)
    snr_db: Optional[float] = None
    quality: float = 1.0         # 0..1 heuristic confidence in this observation


@dataclass
class ThreatResult:
    timestamp: float
    status: str                  # "OK" | "WARN" | "ALERT" | "NO_TARGET"
    side: str                    # "LEFT" | "RIGHT" | "CENTER" | "UNKNOWN"
    urgency: float               # 0..1 (for ramped haptics)
    ttc_s: Optional[float]
    ttr_s: Optional[float]
    target: Optional[Observation]
    debug: Dict[str, float]


# ----------------------------- Provider Interface -----------------------------

class SensorProvider(Protocol):
    """
    Drop-in interface: anything that can produce unified Observations.
    """
    name: str

    def read(self) -> List[Observation]:
        ...


# ----------------------------- Threat Engine -----------------------------

@dataclass
class ThreatConfig:
    # Gating
    rear_angle_gate_deg: float = 20.0          # consider threats within +/- this bearing
    min_range_m: float = 0.5
    max_range_m: float = 150.0

    # TTR (time-to-react) model
    human_reaction_s: float = 0.7
    system_latency_s: float = 0.2

    # Alert thresholds (based on TTR; preferred vs TTC)
    ttr_warn_s: float = 3.0
    ttr_alert_s: float = 1.8

    # Hysteresis / debounce
    consec_on: int = 4
    consec_off: int = 8

    # Smoothing
    ema_alpha: float = 0.25                  # EMA on TTR for stability

    # Urgency ramp range (for haptics)
    urgency_min_ttr: float = 0.6
    urgency_max_ttr: float = 6.0


class ThreatEngine:
    def __init__(self, providers: List[SensorProvider], cfg: ThreatConfig = ThreatConfig()):
        self.providers = providers
        self.cfg = cfg

        # internal state
        self._ttr_ema: float = 99.0
        self._status: str = "OK"
        self._hits_on: int = 0
        self._hits_off: int = 0

    def _valid_candidate(self, o: Observation) -> bool:
        # Must have at least angle and closing speed, and ideally range.
        if o.angle_deg is None:
            return False
        if abs(o.angle_deg) > self.cfg.rear_angle_gate_deg:
            return False

        # If range is known, gate it
        if o.range_m is not None:
            if o.range_m < self.cfg.min_range_m or o.range_m > self.cfg.max_range_m:
                return False

        # Closing speed must be > 0 to be approaching (closing)
        # If unknown, we can't compute TTC/TTR (you could still use "presence" mode later)
        if o.closing_mps is None or o.closing_mps <= 0.0:
            return False

        # If range is unknown, we can still run a proxy TTC using camera size,
        # but that should be encoded by the provider as a pseudo "range_m".
        if o.range_m is None:
            return False

        return True

    def _compute_ttc_ttr(self, o: Observation) -> Tuple[float, float]:
        # TTC = range / closing_speed
        ttc = o.range_m / max(o.closing_mps, 1e-6)
        ttr = ttc - (self.cfg.human_reaction_s + self.cfg.system_latency_s)
        return ttc, ttr

    def _urgency_from_ttr(self, ttr_s: float) -> float:
        # Map TTR -> 0..1 urgency, higher urgency when lower TTR.
        if not math.isfinite(ttr_s):
            return 1.0
        # Normalize into [0,1]
        x = (self.cfg.urgency_max_ttr - ttr_s) / (self.cfg.urgency_max_ttr - self.cfg.urgency_min_ttr)
        x = max(0.0, min(1.0, x))
        # Ease curve: more sensitivity near danger
        return x ** 2.2

    def _side_from_angle(self, angle_deg: Optional[float]) -> str:
        if angle_deg is None:
            return "UNKNOWN"
        if angle_deg < -3.0:
            return "LEFT"
        if angle_deg > 3.0:
            return "RIGHT"
        return "CENTER"

    def step(self) -> ThreatResult:
        now = time.time()

        # 1) Gather observations from all providers
        obs: List[Observation] = []
        for p in self.providers:
            try:
                obs.extend(p.read())
            except Exception:
                # Provider failure shouldn't crash the whole system
                continue

        # 2) Filter to valid threat candidates
        cands = [o for o in obs if self._valid_candidate(o)]

        # 3) Pick the "most dangerous" target (min TTR, then min TTC)
        best: Optional[Observation] = None
        best_ttr = 99.0
        best_ttc = 99.0

        for o in cands:
            ttc, ttr = self._compute_ttc_ttr(o)
            # Prefer smaller ttr; tie-breaker by ttc
            if (ttr < best_ttr) or (abs(ttr - best_ttr) < 1e-3 and ttc < best_ttc):
                best = o
                best_ttr = ttr
                best_ttc = ttc

        # 4) Update EMA + state machine
        if best is None:
            # no target
            self._ttr_ema = min(99.0, self._ttr_ema + 0.2)
            self._hits_on = max(0, self._hits_on - 1)
            self._hits_off += 1
            if self._hits_off >= self.cfg.consec_off:
                self._status = "OK"
            return ThreatResult(
                timestamp=now,
                status="NO_TARGET",
                side="UNKNOWN",
                urgency=0.0,
                ttc_s=None,
                ttr_s=None,
                target=None,
                debug={"candidates": float(len(cands))}
            )

        # Smooth TTR
        self._ttr_ema = (1.0 - self.cfg.ema_alpha) * self._ttr_ema + self.cfg.ema_alpha * best_ttr

        # Determine instantaneous level based on smoothed TTR
        inst = "OK"
        if self._ttr_ema <= self.cfg.ttr_alert_s:
            inst = "ALERT"
        elif self._ttr_ema <= self.cfg.ttr_warn_s:
            inst = "WARN"

        # Debounce / hysteresis
        if inst in ("WARN", "ALERT"):
            self._hits_on += 1
            self._hits_off = max(0, self._hits_off - 1)
            if self._hits_on >= self.cfg.consec_on:
                self._status = inst
        else:
            self._hits_off += 1
            if self._hits_off >= self.cfg.consec_off:
                self._status = "OK"
                self._hits_on = 0

        urgency = self._urgency_from_ttr(self._ttr_ema) if self._status in ("WARN", "ALERT") else 0.0
        side = self._side_from_angle(best.angle_deg)

        return ThreatResult(
            timestamp=now,
            status=self._status,
            side=side,
            urgency=urgency,
            ttc_s=best_ttc,
            ttr_s=self._ttr_ema,
            target=best,
            debug={
                "candidates": float(len(cands)),
                "best_ttc": float(best_ttc),
                "best_ttr_raw": float(best_ttr),
                "ttr_ema": float(self._ttr_ema),
            },
        )


# ----------------------------- Example Providers -----------------------------
# These are minimal examples to show "drop-in" behavior.
# Replace read() with real implementations for radar and camera.

class RadarProviderMock:
    name = "radar_mock"

    def __init__(self):
        self._t0 = time.time()

    def read(self) -> List[Observation]:
        # Simulate one object closing from 60m at 12 m/s, centered
        t = time.time() - self._t0
        range_m = max(2.0, 60.0 - 12.0 * t)
        return [Observation(
            source=self.name,
            obj_id=1,
            timestamp=time.time(),
            range_m=range_m,
            closing_mps=12.0,     # approaching
            angle_deg=-2.0,
            snr_db=18.0,
            quality=1.0
        )]


class CameraProviderMock:
    name = "camera_mock"

    def read(self) -> List[Observation]:
        # Example: camera-only proxy. In a real camera provider you would:
        # - detect + track
        # - estimate bearing angle from pixel center
        # - estimate "range_m" using calibration OR keep it as a rough proxy
        # - estimate closing_mps from change over time
        return []


# ----------------------------- Minimal Demo -----------------------------

def _demo():
    engine = ThreatEngine([RadarProviderMock()], ThreatConfig())
    print("Running threat engine demo (Ctrl+C to stop)")
    while True:
        r = engine.step()
        if r.target:
            print(
                f"{r.status:6s} side={r.side:6s} "
                f"TTC={r.ttc_s:5.2f}s TTR={r.ttr_s:5.2f}s urg={r.urgency:0.2f} "
                f"range={r.target.range_m:5.1f}m closing={r.target.closing_mps:4.1f}m/s angle={r.target.angle_deg:+5.1f}°"
            )
        else:
            print(f"{r.status:6s}")
        time.sleep(0.05)


if __name__ == "__main__":
    _demo()
