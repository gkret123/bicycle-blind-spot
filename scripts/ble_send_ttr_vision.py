#!/usr/bin/env python3
"""
Dual-camera vision + BLE haptics streamer.

Usage:
    # With camera visualization windows
    python scripts/ble_send_ttr_vision.py --show

    # Production (headless, no windows)
    python scripts/ble_send_ttr_vision.py

    # Aggressive L/R split
    python scripts/ble_send_ttr_vision.py --angle-boost 1.0

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

    def stop(self):
        self.vision.stop()


async def run(args):
    vision_cfg = VisionTTRConfig(
        angle_boost=args.angle_boost,
        cam_left_index=args.cam_left,
        cam_right_index=args.cam_right,
        show=args.show,
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
          f"angle_boost={args.angle_boost} | {args.hz}Hz | show={args.show}")

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

    # L/R split
    p.add_argument("--angle-boost", type=float, default=0.7,
                   help="0.0 = symmetric, 1.0 = max directional split")

    # Debug
    p.add_argument("--print-side-values", action="store_true")

    return p.parse_args()


if __name__ == "__main__":
    try:
        asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        print("\nShutdown.")
