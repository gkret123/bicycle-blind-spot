#!/usr/bin/env python3
"""
Fused radar + camera + BLE haptics streamer.

Runs both sensor pipelines simultaneously and blends their urgency outputs
using track-level association and confidence-weighted fusion.

Usage:
    # Production (headless)
    python scripts/ble_send_ttr_fusion.py

    # With camera windows + radar plot
    python scripts/ble_send_ttr_fusion.py --cam-show --radar-show

    # Custom fusion zone boundaries
    python scripts/ble_send_ttr_fusion.py --near-range 4.0 --far-range 20.0

    # Wider track association gate (more aggressive matching)
    python scripts/ble_send_ttr_fusion.py --angle-gate 15

    # Flip radar approaching sign
    python scripts/ble_send_ttr_fusion.py --approaching-sign -1

    # Tune urgency weights
    python scripts/ble_send_ttr_fusion.py --w-proximity 0.5 --w-closing 0.5
"""

import argparse
import asyncio
import os
import time

from bicycle_blind_spot.ble.ble_client import BleAutoReconnectClient, ReconnectPolicy
from bicycle_blind_spot.ble.ttr_streamer import TTRStreamer, StreamConfig
from bicycle_blind_spot.fusion.ttr_source_fusion import FusionTTRSource, FusionTTRConfig
from bicycle_blind_spot.utils.constants import BleTarget
from bicycle_blind_spot.utils.metrics import StreamMetrics


class FusionTTRAdapter:
    """Wraps FusionTTRSource, exposing the same interface as Vision/Radar adapters."""

    def __init__(self, fusion: FusionTTRSource):
        self.fusion = fusion

    def start(self):
        self.fusion.start()

    def value(self) -> float:
        return self.fusion.value()

    def left_ttr(self) -> float:
        return self.fusion.left_ttr

    def right_ttr(self) -> float:
        return self.fusion.right_ttr

    def angle_deg(self) -> float:
        return self.fusion.angle_deg

    def status(self) -> str:
        return self.fusion.status

    def approaching(self) -> bool:
        return self.fusion.approaching

    def fused_label(self) -> str:
        return self.fusion.fused_label

    def step_viz(self):
        """Forward matplotlib update — call from main thread only."""
        self.fusion.step_viz()

    def stop(self):
        self.fusion.stop()


async def run(args):
    fusion_cfg = FusionTTRConfig(
        # Vision
        cam_left_index=args.cam_left,
        cam_right_index=args.cam_right,
        cam_show=args.cam_show or args.show,
        cam_l_offset_deg=-args.cam_angle,
        cam_r_offset_deg=+args.cam_angle,
        h_far=args.h_far,
        h_close=args.h_close,
        dhdt_max=args.dhdt_max,
        min_approach_dhdt=args.min_approach_dhdt,
        # Radar
        cfg_port=args.cfg_port,
        data_port=args.data_port,
        range_preset=args.range_preset,
        approaching_sign=args.approaching_sign,
        radar_show=args.radar_show or args.show,
        verbose_radar=args.verbose_radar,
        range_far=args.range_far,
        range_close=args.range_close,
        closing_max=args.closing_max,
        min_approach_mps=args.min_approach_mps,
        # Fusion
        near_range_m=args.near_range,
        far_range_m=args.far_range,
        angle_gate_deg=args.angle_gate,
        max_time_delta_s=args.max_time_delta,
        fusion_strategy=args.fusion_strategy,
        # Shared
        ttr_smooth_alpha=args.ttr_smooth,
        both_zone_deg=args.both_zone,
    )
    source = FusionTTRAdapter(FusionTTRSource(fusion_cfg))

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
    print("Both devices connected")

    # Start fusion (configures radar + starts all sensor threads)
    source.start()
    print("Fusion system started (radar + vision)")

    if args.notify:
        async def on_notify(side: str, data: bytes):
            print(f"[notify:{side}] {data.decode('utf-8', errors='replace')}")
        await asyncio.gather(*(
            c.start_notify(lambda d, s=side: on_notify(s, d))
            for side, c in clients.items()
        ))

    # Per-device transforms pull the angle-split TTR from the fused source
    transforms = {
        clients["left"].target.device_name:  lambda _: source.left_ttr(),
        clients["right"].target.device_name: lambda _: source.right_ttr(),
    }

    labels = {
        clients["left"].target.device_name:  f"left:{clients['left'].target.device_name}",
        clients["right"].target.device_name: f"right:{clients['right'].target.device_name}",
    }

    class FusionMetrics(StreamMetrics):
        def maybe_print(self, **kwargs):
            now = time.monotonic()
            if now - self.last_print_at < self.print_every_s:
                return
            self.last_print_at = now
            elapsed = now - self.started_at
            hz = (self.writes_total / elapsed) if elapsed > 0 else 0.0
            states = kwargs.get("states", {})
            label = source.fused_label()
            print(
                f"[{elapsed:6.1f}s] base={kwargs.get('ttr', 0.0):.3f} "
                f"L={source.left_ttr():.3f} R={source.right_ttr():.3f} "
                f"angle={source.angle_deg():+.1f} status={source.status()} "
                f"src={label} "
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
        metrics=FusionMetrics(print_every_s=args.print_every),
        transforms=transforms,
        labels=labels,
    )

    print(f"Streaming FUSION | "
          f"mode={args.fusion_strategy} | "
          f"near={args.near_range}m far={args.far_range}m | "
          f"angle_gate={args.angle_gate} | "
          f"both_zone={args.both_zone} | "
          f"{args.hz}Hz")

    async def _viz_loop():
        """Drive matplotlib from the main (asyncio) thread at ~20 Hz."""
        while True:
            source.step_viz()
            await asyncio.sleep(0.05)

    tasks = [asyncio.create_task(streamer.start())]
    if args.radar_show or args.cam_show or args.show:
        tasks.append(asyncio.create_task(_viz_loop()))

    try:
        await asyncio.gather(*tasks)
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*(c.disconnect() for c in clients.values()))
        source.stop()
        print("Disconnected.")


def parse_args():
    p = argparse.ArgumentParser(
        description="Fused radar+camera TTR sender for left/right BLE haptic units."
    )

    # ---- Camera ----
    cam = p.add_argument_group("camera")
    cam.add_argument("--cam-left",  type=int, default=0)
    cam.add_argument("--cam-right", type=int, default=1)
    cam.add_argument("--cam-angle", type=float, default=35.0,
                     help="Camera mount angle from straight-back (default: 35)")
    cam.add_argument("--cam-show", action="store_true",
                     help="Show cv2.imshow camera windows")
    p.add_argument("--show", action="store_true",
                   help="Enable all visualizations (camera windows + radar scatter plot)")

    # ---- Radar ----
    radar = p.add_argument_group("radar")
    radar.add_argument("--cfg-port",   default="/dev/ttyUSB0")
    radar.add_argument("--data-port",  default="/dev/ttyUSB1")
    radar.add_argument("--range-preset", default="long", choices=["standard", "long"])
    radar.add_argument("--approaching-sign", type=int, default=1, choices=[-1, 1])
    radar.add_argument("--radar-show", action="store_true",
                       help="Show live top-down radar scatter plot")
    radar.add_argument("--verbose-radar", action="store_true")

    # ---- BLE ----
    ble = p.add_argument_group("BLE")
    ble.add_argument("--left-device",  default="BBSpot-XIAO-L")
    ble.add_argument("--right-device", default="BBSpot-XIAO-R")
    ble.add_argument("--scan-timeout", type=float, default=8.0)
    ble.add_argument("--notify", action="store_true")

    # ---- Stream ----
    p.add_argument("--hz",          type=float, default=20.0)
    p.add_argument("--print-every", type=float, default=0.5)

    # ---- Fusion parameters ----
    fusion = p.add_argument_group("fusion",
        "Controls how radar and camera urgency are blended. "
        "At near range camera dominates; at far range radar dominates; "
        "in between the weights crossfade linearly.")
    fusion.add_argument("--near-range", type=float, default=5.0,
                        help="Below this range (m), camera dominates (default: 5)")
    fusion.add_argument("--far-range",  type=float, default=15.0,
                        help="Above this range (m), radar dominates (default: 15)")
    fusion.add_argument("--angle-gate", type=float, default=12.0,
                        help="Max bearing diff (deg) to associate radar+vision tracks (default: 12)")
    fusion.add_argument("--max-time-delta", type=float, default=0.3,
                        help="Max time gap (s) between sensor readings for association (default: 0.3)")
    fusion.add_argument(
        "--fusion-strategy",
        default="confidence_weighted",
        choices=["confidence_weighted", "track_level"],
        help="Fusion strategy: confidence_weighted (recommended start) "
             "or track_level (most robust association)."
    )

    # ---- Urgency tuning (shared) ----
    urgency = p.add_argument_group("urgency tuning")
    # Vision
    urgency.add_argument("--h-far",       type=float, default=0.05)
    urgency.add_argument("--h-close",     type=float, default=0.45)
    urgency.add_argument("--dhdt-max",    type=float, default=40.0)
    urgency.add_argument("--min-approach-dhdt", type=float, default=0.5)
    # Radar
    urgency.add_argument("--range-far",   type=float, default=80.0)
    urgency.add_argument("--range-close", type=float, default=3.0)
    urgency.add_argument("--closing-max", type=float, default=15.0)
    urgency.add_argument("--min-approach-mps", type=float, default=0.5)
    # Smoothing
    urgency.add_argument("--ttr-smooth",  type=float, default=0.15)

    # ---- Side selection ----
    p.add_argument("--both-zone", type=float, default=10.0,
                   help="Angle zone (deg) where both sides vibrate (default: 10)")

    # ---- Debug ----
    p.add_argument("--print-side-values", action="store_true")

    return p.parse_args()


if __name__ == "__main__":
    os.environ.setdefault(
        "QT_LOGGING_RULES",
        "qt.qpa.wayland=false;qt.qpa.window=false",
    )
    try:
        asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        print("\nShutdown.")
