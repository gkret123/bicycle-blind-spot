# scripts/ble_send_ttr.py
 
import argparse
import asyncio
from dataclasses import dataclass

from bicycle_blind_spot.ble.ble_client import BleAutoReconnectClient, ReconnectPolicy
from bicycle_blind_spot.ble.ttr_streamer import TTRStreamer, StreamConfig
from bicycle_blind_spot.radar.ttr_source import TTRRamp
from bicycle_blind_spot.utils.constants import BleTarget
from bicycle_blind_spot.utils.math_utils import clamp01
from bicycle_blind_spot.utils.metrics import StreamMetrics


@dataclass
class SideSignalConfig:
    scale: float = 1.0
    offset: float = 0.0

    def apply(self, ttr: float) -> float:
        return clamp01(ttr * self.scale + self.offset)


async def run(args):
    source = TTRRamp(
        ramp_seconds=args.ramp_s,
        hold_seconds=args.hold_s,
    )
    metrics = StreamMetrics(print_every_s=args.print_every)

    clients = {
        "left": BleAutoReconnectClient(
            BleTarget(args.left_device),
            policy=ReconnectPolicy(scan_timeout_s=args.scan_timeout),
            on_reconnect=metrics.note_reconnect,
        ),
        "right": BleAutoReconnectClient(
            BleTarget(args.right_device),
            policy=ReconnectPolicy(scan_timeout_s=args.scan_timeout),
            on_reconnect=metrics.note_reconnect,
        ),
    }

    side_cfg = {
        "left": SideSignalConfig(
            scale=args.left_scale,
            offset=args.left_offset,
        ),
        "right": SideSignalConfig(
            scale=args.right_scale,
            offset=args.right_offset,
        ),
    }

    async def on_notify(side: str, data: bytes):
        msg = data.decode("utf-8", errors="replace")
        print(f"[notify:{side}] {msg}")

    # Connect explicitly before enabling notifications so startup is predictable.
    await asyncio.gather(*(client.ensure_connected() for client in clients.values()))

    if args.notify:
        await asyncio.gather(
            *(
                client.start_notify(lambda data, side=side: on_notify(side, data))
                for side, client in clients.items()
            )
        )

    transforms = {
        clients["left"].target.device_name: side_cfg["left"].apply,
        clients["right"].target.device_name: side_cfg["right"].apply,
    }

    labels = {
        clients["left"].target.device_name: f"left:{clients['left'].target.device_name}",
        clients["right"].target.device_name: f"right:{clients['right'].target.device_name}",
    }

    streamer = TTRStreamer(
        source=source,
        clients=list(clients.values()),
        cfg=StreamConfig(
            hz=args.hz,
            stop_at_end=args.stop_at_end,
            print_per_client_values=args.print_side_values,
        ),
        metrics=metrics,
        transforms=transforms,
        labels=labels,
    )

    print("Streaming TTR to left/right ESP32 units...")
    try:
        await streamer.start()
    finally:
        await asyncio.gather(*(client.disconnect() for client in clients.values()))
        print("Disconnected.")


def parse_args():
    p = argparse.ArgumentParser(
        description="Dual-stream TTR sender for left/right BLE bicycle blind-spot units."
    )

    # Dual-stream defaults
    p.add_argument("--left-device", default="BBSpot-XIAO-L")
    p.add_argument("--right-device", default="BBSpot-XIAO-R")

    # BLE / stream timing
    p.add_argument("--scan-timeout", type=float, default=8.0)
    p.add_argument("--hz", type=float, default=20.0)
    p.add_argument("--ramp-s", type=float, default=45.0)
    p.add_argument("--hold-s", type=float, default=10.0)
    p.add_argument("--stop-at-end", action="store_true")
    p.add_argument("--notify", action="store_true")
    p.add_argument("--print-every", type=float, default=0.5)

    # Per-side shaping
    p.add_argument(
        "--left-scale",
        type=float,
        default=1.0,
        help="Multiply base TTR for the left unit before clamping to [0,1].",
    )
    p.add_argument(
        "--left-offset",
        type=float,
        default=0.0,
        help="Add offset to left-unit TTR after scaling, then clamp to [0,1].",
    )
    p.add_argument(
        "--right-scale",
        type=float,
        default=1.0,
        help="Multiply base TTR for the right unit before clamping to [0,1].",
    )
    p.add_argument(
        "--right-offset",
        type=float,
        default=0.0,
        help="Add offset to right-unit TTR after scaling, then clamp to [0,1].",
    )

    # Debug
    p.add_argument(
        "--print-side-values",
        action="store_true",
        help="Print base/left/right TTR and byte values every loop.",
    )

    return p.parse_args()


if __name__ == "__main__":
    asyncio.run(run(parse_args()))