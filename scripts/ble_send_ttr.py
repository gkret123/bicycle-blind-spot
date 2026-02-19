# scripts/ble_send_ttr.py
import argparse
import asyncio

from bicycle_blind_spot.ttr_source import TTRRamp
from bicycle_blind_spot.constants import BleTarget
from bicycle_blind_spot.ble_client import BleAutoReconnectClient, ReconnectPolicy
from bicycle_blind_spot.ttr_streamer import TTRStreamer, StreamConfig
from bicycle_blind_spot.metrics import StreamMetrics


async def run(args):
    # TTR ramp source (placeholder until real TTR is computed)
    source = TTRRamp(ramp_seconds=args.ramp_s, hold_seconds=args.hold_s)

    # Metrics (prints periodically)
    metrics = StreamMetrics(print_every_s=args.print_every)

    # Build BLE clients
    targets = [BleTarget(name.strip()) for name in args.device_names.split(",") if name.strip()]

    def on_reconnect(device_name: str) -> None:
        metrics.note_reconnect(device_name)

    policy = ReconnectPolicy(scan_timeout_s=args.scan_timeout)
    clients = [BleAutoReconnectClient(t, policy=policy, on_reconnect=on_reconnect) for t in targets]

    # Optional TX notifications (ESP32 -> Pi). Good for debugging.
    async def on_notify(data: bytes):
        print("[notify]", data.decode("utf-8", errors="replace"))

    if args.notify:
        await asyncio.gather(*(c.start_notify(on_notify) for c in clients))

    streamer = TTRStreamer(
        source=source,
        clients=clients,
        cfg=StreamConfig(hz=args.hz, stop_at_end=args.stop_at_end),
        metrics=metrics,
    )

    print("Streaming TTR...")
    try:
        await streamer.start()
    finally:
        await asyncio.gather(*(c.disconnect() for c in clients))
        print("Disconnected.")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--device-names", default="BBSpot-XIAO",
                   help='Comma-separated, e.g. "BBSpot-XIAO-L,BBSpot-XIAO-R"')
    p.add_argument("--scan-timeout", type=float, default=8.0)
    p.add_argument("--hz", type=float, default=20.0)
    p.add_argument("--ramp-s", type=float, default=45.0)
    p.add_argument("--hold-s", type=float, default=10.0)
    p.add_argument("--stop-at-end", action="store_true")
    p.add_argument("--notify", action="store_true")
    p.add_argument("--print-every", type=float, default=0.5)
    return p.parse_args()


if __name__ == "__main__":
    asyncio.run(run(parse_args()))
