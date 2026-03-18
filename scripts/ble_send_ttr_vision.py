#!/usr/bin/env python3
"""
Dual-camera vision + BLE haptics streamer.

Usage:
    # With camera visualization windows
    python scripts/ble_send_ttr_vision.py --show

    # Production (headless, no windows)
    python scripts/ble_send_ttr_vision.py

    # Custom camera mount angles (e.g. 25° instead of default 35°)
    python scripts/ble_send_ttr_vision.py --cam-angle 25

    # Wider "both-sides" vibration zone (±15° instead of default ±10°)
    python scripts/ble_send_ttr_vision.py --both-zone 15

    # Tune urgency weights (more proximity-driven)
    python scripts/ble_send_ttr_vision.py --w-proximity 0.6 --w-closing 0.4

    # Print per-device values every loop
    python scripts/ble_send_ttr_vision.py --print-side-values
"""

import argparse
import asyncio
import time
from dataclasses import dataclass

from bicycle_blind_spot.ble.ble_client import BleAutoReconnectClient, ReconnectPolicy
from bicycle_blind_spot.ble.ttr_streamer import TTRStreamer, StreamConfig
from bicycle_blind_spot.vision.ttr_source_vision import VisionTTRSource, VisionTTRConfig
from bicycle_blind_spot.utils.constants import BleTarget
from bicycle_blind_spot.utils.metrics import StreamMetrics


class VisionTTRAdapter:
    """Wraps VisionTTRSource, exposing .value() for TTRStreamer and per-side TTR for transforms."""

    def __init__(self, vision: VisionTTRSource):
        self.vision = vision

    def start(self):
        self.vision.start()

    def value(self) -> float:
        return self.vision.value()

    def left_ttr(self) -> float:
        return self.vision.left_ttr

    def right_ttr(self) -> float:
        return self.vision.right_ttr

    def angle_deg(self) -> float:
        return self.vision.angle_deg

    def status(self) -> str:
        return self.vision.status

    def approaching(self) -> bool:
        return self.vision.approaching

    def stop(self):
        self.vision.stop()


async def run(args):
    vision_cfg = VisionTTRConfig(
        cam_left_index=args.cam_left,
        cam_right_index=args.cam_right,
        show=args.show,
        cam_l_offset_deg=-args.cam_angle,
        cam_r_offset_deg=+args.cam_angle,
        # Urgency weights
        w_proximity=args.w_proximity,
        w_closing=args.w_closing,
        h_far=args.h_far,
        h_close=args.h_close,
        dhdt_max=args.dhdt_max,
        # Approach gate & smoothing
        min_approach_dhdt=args.min_approach_dhdt,
        ttr_smooth_alpha=args.ttr_smooth,
        # Side selection
        both_zone_deg=args.both_zone,
    )
    source = VisionTTRAdapter(VisionTTRSource(vision_cfg))

    clients = {
        "left": BleAutoReconnectClient(
            BleTarget(args.left_device),
            policy=ReconnectPolicy(scan_timeout_s=args.scan_timeout),
        ),
        "right": BleAutoReconnectClient(
            BleTarget(args.right_device),
            policy=ReconnectPolicy(scan_timeout_s=args.scan_timeout),
        ),
    }

    print(f"Connecting to {args.left_device} and {args.right_device}...")
    await asyncio.gather(*(c.ensure_connected() for c in clients.values()))
    print("✓ Both devices connected")

    # Start vision AFTER BLE is connected so we don't waste cycles
    # while waiting for devices.
    source.start()
    print("✓ Vision system started")

    if args.notify:
        async def on_notify(side: str, data: bytes):
            print(f"[notify:{side}] {data.decode('utf-8', errors='replace')}")
        await asyncio.gather(*(
            c.start_notify(lambda d, s=side: on_notify(s, d))
            for side, c in clients.items()
        ))

    # Per-device transforms pull the angle-split TTR from the vision source
    transforms = {
        clients["left"].target.device_name:  lambda _: source.left_ttr(),
        clients["right"].target.device_name: lambda _: source.right_ttr(),
    }

    labels = {
        clients["left"].target.device_name:  f"left:{clients['left'].target.device_name}",
        clients["right"].target.device_name: f"right:{clients['right'].target.device_name}",
    }

    class VisionMetrics(StreamMetrics):
        def maybe_print(self, **kwargs):
            now = time.monotonic()
            if now - self.last_print_at < self.print_every_s:
                return
            self.last_print_at = now
            elapsed = now - self.started_at
            hz = (self.writes_total / elapsed) if elapsed > 0 else 0.0
            states = kwargs.get("states", {})
            print(
                f"[{elapsed:6.1f}s] base={kwargs.get('ttr', 0.0):.3f} "
                f"L={source.left_ttr():.3f} R={source.right_ttr():.3f} "
                f"angle={source.angle_deg():+.1f}° status={source.status()} "
                f"hz={hz:.1f} "
                + " ".join(f"{k}:{v}" for k, v in states.items())
            )

    streamer = TTRStreamer(
        source=source,
        clients=list(clients.values()),
        cfg=StreamConfig(
            hz=args.hz,
            print_per_client_values=args.print_side_values,
        ),
        metrics=VisionMetrics(print_every_s=args.print_every),
        transforms=transforms,
        labels=labels,
    )

    print(f"Streaming — cameras L@{args.cam_left} R@{args.cam_right} | "
          f"cam_angle=±{args.cam_angle}° | both_zone=±{args.both_zone}° | "
          f"w_prox={args.w_proximity} w_close={args.w_closing} | "
          f"{args.hz}Hz | show={args.show}")

    try:
        await streamer.start()
    finally:
        await asyncio.gather(*(c.disconnect() for c in clients.values()))
        source.stop()
        print("Disconnected.")


def parse_args():
    p = argparse.ArgumentParser(
        description="Dual-camera vision TTR sender for left/right BLE haptic units."
    )

    # Camera
    p.add_argument("--cam-left",  type=int, default=0)
    p.add_argument("--cam-right", type=int, default=1)
    p.add_argument("--cam-angle", type=float, default=35.0,
                   help="Camera mount angle in degrees from straight-back (default: 35)")
    p.add_argument("--show", action="store_true",
                   help="Show cv2.imshow camera windows (omit for headless/production)")

    # BLE devices
    p.add_argument("--left-device",  default="BBSpot-XIAO-L")
    p.add_argument("--right-device", default="BBSpot-XIAO-R")
    p.add_argument("--scan-timeout", type=float, default=8.0)
    p.add_argument("--notify", action="store_true")

    # Stream
    p.add_argument("--hz",          type=float, default=20.0)
    p.add_argument("--print-every", type=float, default=0.5)

    # TTR urgency tuning
    urgency = p.add_argument_group("urgency tuning",
        "Weights and thresholds for the two-factor TTR model. "
        "TTR = 1 - clamp01(w_proximity * proximity + w_closing * closing).")
    urgency.add_argument("--w-proximity", type=float, default=0.4,
                         help="Weight for proximity (bbox size) in urgency (default: 0.4)")
    urgency.add_argument("--w-closing",   type=float, default=0.6,
                         help="Weight for closing speed (bbox growth) in urgency (default: 0.6)")
    urgency.add_argument("--h-far",       type=float, default=0.05,
                         help="Bbox-height/frame-height ratio below which proximity=0 (default: 0.05)")
    urgency.add_argument("--h-close",     type=float, default=0.45,
                         help="Bbox-height/frame-height ratio above which proximity=1 (default: 0.45)")
    urgency.add_argument("--dhdt-max",    type=float, default=40.0,
                         help="Bbox growth rate (px/s) for maximum closing urgency (default: 40)")
    urgency.add_argument("--min-approach-dhdt", type=float, default=0.5,
                         help="Min bbox growth rate (px/s) to count as approaching. "
                              "Below this = no vibration. (default: 0.5)")
    urgency.add_argument("--ttr-smooth",  type=float, default=0.15,
                         help="EMA alpha for smoothing TTR output. "
                              "Lower = smoother, higher = snappier. (default: 0.15)")

    # Side selection
    p.add_argument("--both-zone", type=float, default=10.0,
                   help="Angle zone (±degrees) where both sides vibrate (default: 10)")

    # Debug
    p.add_argument("--print-side-values", action="store_true")

    return p.parse_args()


if __name__ == "__main__":
    try:
        asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        print("\nShutdown.")
