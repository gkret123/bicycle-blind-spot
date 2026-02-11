import asyncio
import sys
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "BBSpot-XIAO"

SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
RX_UUID      = "12345678-1234-1234-1234-1234567890ac"  # Write to ESP
TX_UUID      = "12345678-1234-1234-1234-1234567890ad"  # Notify from ESP

SCAN_TIMEOUT_S = 8.0
RECONNECT_DELAY_S = 1.0


def on_notify(sender: int, data: bytearray):
    txt = data.decode("utf-8", errors="replace")
    print(f"\n[ESP NOTIFY] {txt}\n> ", end="", flush=True)


def has_service_uuid(advertisement_data) -> bool:
    """Best-effort check for service UUID in ad data (works across Bleak versions)."""
    if advertisement_data is None:
        return False
    uuids = getattr(advertisement_data, "service_uuids", None)
    if not uuids:
        return False
    return SERVICE_UUID.lower() in [u.lower() for u in uuids]


async def find_device_address() -> str:
    print(f"Scanning for {DEVICE_NAME} (or service {SERVICE_UUID}) ...")

    def _filter(device, adv):
        # Prefer service UUID match if present, otherwise fall back to name
        if has_service_uuid(adv):
            return True
        name = device.name or ""
        return name == DEVICE_NAME

    dev = await BleakScanner.find_device_by_filter(_filter, timeout=SCAN_TIMEOUT_S)

    if dev is None:
        raise RuntimeError(f"Could not find {DEVICE_NAME}. Is it powered on and advertising?")

    print(f"Found: {dev.name} [{dev.address}]")
    return dev.address


async def interactive_loop(client: BleakClient):
    print("Connected. Type commands: ping | buzz | effect:47 | stop | quit")
    print("> ", end="", flush=True)

    loop = asyncio.get_running_loop()
    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if not line:
            continue

        cmd = line.strip()
        if cmd == "":
            print("> ", end="", flush=True)
            continue

        if cmd.lower() in ("q", "quit", "exit"):
            return

        # Write to RX characteristic
        await client.write_gatt_char(RX_UUID, cmd.encode("utf-8"), response=True)
        print(f"[PI -> ESP] {cmd}")
        print("> ", end="", flush=True)


async def run_forever():
    address = None

    while True:
        try:
            if address is None:
                address = await find_device_address()

            print(f"Connecting to {address} ...")
            async with BleakClient(address) as client:
                if not client.is_connected:
                    raise RuntimeError("Connect failed (not connected).")

                print("✅ Connected")

                await client.start_notify(TX_UUID, on_notify)
                print("🔔 Notifications enabled")

                # quick handshake
                await client.write_gatt_char(RX_UUID, b"ping", response=True)

                await interactive_loop(client)

        except Exception as e:
            print(f"\n[DISCONNECTED / RETRY] {e}")
            print(f"Retrying in {RECONNECT_DELAY_S}s...\n")
            await asyncio.sleep(RECONNECT_DELAY_S)

            # If the ESP's address changes (rare), rescan
            # address = None


if __name__ == "__main__":
    try:
        asyncio.run(run_forever())
    except KeyboardInterrupt:
        print("\nBye.")
