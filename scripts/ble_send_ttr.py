# scripts/ble_send_ttr.py
import argparse
import asyncio

from src.ttr_source import TTRRamp
from src.ble_haptics import BleConfig, BleHapticsClient


SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
RX_UUID      = "12345678-1234-1234-1234-1234567890ac"
TX_UUID      = "12345678-1234-1234-1234-1234567890ad"


async def run(args):
    ttr = TTRRamp(ramp_seconds=args.ramp_s, hold_seconds=args.hold_s)

    client = BleHapticsClient(
        BleConfig(
            device_name=args.device_name,
            service_uuid=SERVICE_UUID,
            rx_uuid=RX_UUID,
            tx_uuid=TX_UUID,
            scan_timeout_s=args.scan_timeout,
        )
    )

    print("Connecting...")
    await client.connect()
    print("✅ Connected")

    try:
        period = 1.0 / args.hz
        while True:
            v = ttr.value()
            await client.send_ttr_scalar(v, response=False)

            if args.print:
                print(f"ttr={v:0.3f}")

            if args.stop_at_end and v <= 0.0:
                break

            await asyncio.sleep(period)

        await client.send_ttr_scalar(0.0, response=False)
    finally:
        await client.disconnect()
        print("Disconnected.")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--device-name", default="BBSpot-XIAO")
    p.add_argument("--scan-timeout", type=float, default=8.0)
    p.add_argument("--hz", type=float, default=20.0)
    p.add_argument("--ramp-s", type=float, default=45.0)
    p.add_argument("--hold-s", type=float, default=10.0)
    p.add_argument("--print", action="store_true")
    p.add_argument("--stop-at-end", action="store_true")
    return p.parse_args()


if __name__ == "__main__":
    asyncio.run(run(parse_args()))
