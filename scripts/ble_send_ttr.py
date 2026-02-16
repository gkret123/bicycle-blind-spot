# scripts/ble_send_ttr.py

import argparse
import asyncio

from src.constants import BleTarget
from src.ble_client import BleAutoReconnectClient, ReconnectPolicy
from src.ttr_source import TTRRamp
from src.ttr_streamer import TTRStreamer, StreamConfig


async def run(args):
    source = TTRRamp(ramp_seconds=args.ramp_s)

    targets = [BleTarget(name.strip()) for name in args.device_names.split(",")]
    clients = [BleAutoReconnectClient(t, ReconnectPolicy(scan_timeout_s=args.scan_timeout))
               for t in targets]

    streamer = TTRStreamer(source, clients, StreamConfig(hz=args.hz))

    print("Streaming TTR...")
    await streamer.start()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--device-names", default="BBSpot-XIAO")
    p.add_argument("--scan-timeout", type=float, default=8.0)
    p.add_argument("--hz", type=float, default=20.0)
    p.add_argument("--ramp-s", type=float, default=45.0)
    return p.parse_args()


if __name__ == "__main__":
    asyncio.run(run(parse_args()))
