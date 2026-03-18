#!/usr/bin/env python3
"""
Radar + BLE haptics streamer.

Usage:
    # Production (headless)
    python scripts/ble_send_ttr_radar.py

    # Long-range preset (default, ~90m)
    python scripts/ble_send_ttr_radar.py --range-preset long

    # Standard range preset (~27m, original config)
    python scripts/ble_send_ttr_radar.py --range-preset standard

    # Tune urgency weights (more proximity-driven)
    python scripts/ble_send_ttr_radar.py --w-proximity 0.6 --w-closing 0.4

    # Wider "both-sides" vibration zone (±15° instead of default ±10°)
    python scripts/ble_send_ttr_radar.py --both-zone 15

    # Print per-device values every loop
    python scripts/ble_send_ttr_radar.py --print-side-values

    # Flip approaching sign if radar reports approach as negative v_r
    python scripts/ble_send_ttr_radar.py --approaching-sign -1
"""

import argparse
import asyncio
import time

from bicycle_blind_spot.ble.ble_client import BleAutoReconnectClient, ReconnectPolicy
from bicycle_blind_spot.ble.ttr_streamer import TTRStreamer, StreamConfig
from bicycle_blind_spot.radar.ttr_source_radar import RadarTTRSource, RadarTTRConfig
from bicycle_blind_spot.utils.constants import BleTarget
from bicycle_blind_spot.utils.metrics import StreamMetrics


class RadarTTRAdapter:
    """Wraps RadarTTRSource, exposing .value() for TTRStreamer and per-side TTR for transforms."""

    def __init__(self, radar: RadarTTRSource):
        self.radar = radar

    def start(self):
        self.radar.start()

    def value(self) -> float:
        return self.radar.value()

    def left_ttr(self) -> float:
        return self.radar.left_ttr

    def right_ttr(self) -> float:
        return self.radar.right_ttr

    def angle_deg(self) -> float:
        return self.radar.angle_deg

    def status(self) -> str:
        return self.radar.status

    def approaching(self) -> bool:
        return self.radar.approaching

    def step_viz(self):
        """Forward matplotlib update — call from main thread only."""
        self.radar.step_viz()

    def stop(self):
        self.radar.stop()


async def run(args):
    radar_cfg = RadarTTRConfig(
        cfg_port=args.cfg_port,
        data_port=args.data_port,
        range_preset=args.range_preset,
        approaching_sign=args.approaching_sign,
        # Urgency weights
        w_proximity=args.w_proximity,
        w_closing=args.w_closing,
        range_far=args.range_far,
        range_close=args.range_close,
        closing_max=args.closing_max,
        # Approach gate & smoothing
        min_approach_mps=args.min_approach_mps,
        ttr_smooth_alpha=args.ttr_smooth,
        # Side selection
        both_zone_deg=args.both_zone,
        # Visualization
        show=args.show,
    )
    source = RadarTTRAdapter(RadarTTRSource(radar_cfg))

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

    # Start radar AFTER BLE is connected so we don't waste cycles
    # while waiting for devices.
    source.start()
    print("✓ Radar system started")

    if args.notify:
        async def on_notify(side: str, data: bytes):
            print(f"[notify:{side}] {data.decode('utf-8', errors='replace')}")
        await asyncio.gather(*(
            c.start_notify(lambda d, s=side: on_notify(s, d))
            for side, c in clients.items()
        ))

    # Per-device transforms pull the angle-split TTR from the radar source
    transforms = {
        clients["left"].target.device_name:  lambda _: source.left_ttr(),
        clients["right"].target.device_name: lambda _: source.right_ttr(),
    }

    labels = {
        clients["left"].target.device_name:  f"left:{clients['left'].target.device_name}",
        clients["right"].target.device_name: f"right:{clients['right'].target.device_name}",
    }

    class RadarMetrics(StreamMetrics):
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
        metrics=RadarMetrics(print_every_s=args.print_every),
        transforms=transforms,
        labels=labels,
    )

    print(f"Streaming — preset={args.range_preset} | "
          f"both_zone=±{args.both_zone}° | "
          f"w_prox={args.w_proximity} w_close={args.w_closing} | "
          f"{args.hz}Hz")

    async def _viz_loop():
        """Drive matplotlib from the main (asyncio) thread at ~20 Hz."""
        while True:
            source.step_viz()
            await asyncio.sleep(0.05)

    tasks = [asyncio.create_task(streamer.start())]
    if args.show:
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
        description="Radar TTR sender for left/right BLE haptic units."
    )

    # Radar hardware
    p.add_argument("--cfg-port",   default="/dev/ttyUSB0",
                   help="Radar CLI/config serial port (default: /dev/ttyUSB0)")
    p.add_argument("--data-port",  default="/dev/ttyUSB1",
                   help="Radar data serial port (default: /dev/ttyUSB1)")
    p.add_argument("--range-preset", default="long", choices=["standard", "long"],
                   help="Radar range preset: 'standard' (~27m) or 'long' (~90m, default)")
    p.add_argument("--approaching-sign", type=int, default=1, choices=[-1, 1],
                   help="Sign of v_r for approaching vehicles: +1 or -1 (calibrate on first test)")

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
                         help="Weight for proximity (range) in urgency (default: 0.4)")
    urgency.add_argument("--w-closing",   type=float, default=0.6,
                         help="Weight for closing speed in urgency (default: 0.6)")
    urgency.add_argument("--range-far",   type=float, default=80.0,
                         help="Range (m) beyond which proximity=0 (default: 80)")
    urgency.add_argument("--range-close", type=float, default=3.0,
                         help="Range (m) within which proximity=1 (default: 3)")
    urgency.add_argument("--closing-max", type=float, default=15.0,
                         help="Closing speed (m/s) for max closing urgency (default: 15)")
    urgency.add_argument("--min-approach-mps", type=float, default=0.5,
                         help="Min closing speed (m/s) to count as approaching. "
                              "Below this = no vibration. (default: 0.5)")
    urgency.add_argument("--ttr-smooth",  type=float, default=0.15,
                         help="EMA alpha for smoothing TTR output (default: 0.15)")

    # Side selection
    p.add_argument("--both-zone", type=float, default=10.0,
                   help="Angle zone (±degrees) where both sides vibrate (default: 10)")

    # Visualization
    p.add_argument("--show", action="store_true",
                   help="Show live top-down radar scatter plot (requires display)")

    # Debug
    p.add_argument("--print-side-values", action="store_true")

    return p.parse_args()


if __name__ == "__main__":
    try:
        asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        print("\nShutdown.")
