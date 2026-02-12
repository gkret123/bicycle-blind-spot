import time
import struct
import argparse

import serial
import numpy as np
import matplotlib.pyplot as plt

MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"

CMDS = [
    "sensorStop",
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 15 5 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "lowPower 0 0",
    "profileCfg 0 77 7 3 39 0 0 100 1 256 7200 0 0 30",
    "chirpCfg 0 0 0 0 0 0 0 1",
    "chirpCfg 1 1 0 0 0 0 0 4",
    "frameCfg 0 1 32 0 100 1 0",
    "guiMonitor -1 1 1 1 0 0 1",
    "cfarCfg -1 0 2 8 4 3 0 15.0 0",
    "cfarCfg -1 1 0 4 2 3 1 15.0 0",
    "multiObjBeamForming -1 1 0.5",
    "calibDcRangeSig -1 0 -5 8 256",
    "clutterRemoval -1 0",
    "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0",
    "measureRangeBiasAndRxChanPhase 0 1. 0.2",
    "aoaFovCfg -1 -90 90 -90 90",
    "cfarFovCfg -1 0 0.25 8.64",
    "cfarFovCfg -1 1 -10.59 10.59",
    "extendedMaxVelocity -1 0",
    "CQRxSatMonitor 0 3 11 121 0",
    "CQSigImgMonitor 0 127 8",
    "analogMonitor 0 0",
    "lvdsStreamCfg -1 0 0 0",
    "calibData 0 0 0",
    "sensorStart",
]

def is_hard_error(line: str) -> bool:
    l = line.lower()
    return ("error" in l) or ("invalid" in l) or ("unknown" in l) or ("not recognized" in l)

def send_cmd_wait_done(ser: serial.Serial, cmd: str, timeout_s: float) -> bool:
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()

    t0 = time.time()
    saw_any = False

    while time.time() - t0 < timeout_s:
        line = ser.readline()
        if not line:
            continue
        s = line.decode(errors="ignore").strip()
        if not s:
            continue

        saw_any = True
        print(f"  {s}")

        sl = s.lower()
        if "ignored:" in sl and ("already stopped" in sl or "already started" in sl):
            return True

        if is_hard_error(s):
            return False

        if "Done" in s:
            return True

    if cmd == "sensorStop" and saw_any:
        return True
    return False

def configure_sensor(cfg_port: str):
    print(f"\n== Configuring sensor on {cfg_port} @ 115200 ==")
    with serial.Serial(cfg_port, 115200, timeout=0.2, write_timeout=1) as s:
        time.sleep(0.25)
        for c in CMDS:
            print(f"\n> {c}")
            to = 4.0 if c == "sensorStart" else 1.5
            if not send_cmd_wait_done(s, c, timeout_s=to):
                raise RuntimeError(f"Command failed: {c}")
    print("\n✅ Sensor configured / started.")

def parse_tlv1_xy(payload: bytes):
    """
    Your firmware's TLV1 payload is float32 x,y,z,vel repeated, PLUS 8 extra bytes.
    So handle:
      - L % 16 == 0
      - (L - 8) % 16 == 0   <-- what your probe showed consistently
    Returns Nx2 float32 (x,y) or None.
    """
    L = len(payload)
    for trim in (0, 8, 4, 12):  # try common “extra bytes” sizes
        if L > trim and ((L - trim) % 16) == 0:
            body = payload[: L - trim]
            pts = np.frombuffer(body, dtype=np.float32)
            if pts.size >= 4 and (pts.size % 4) == 0:
                pts = pts.reshape(-1, 4)
                xy = pts[:, :2]
                # basic sanity: finite points only
                mask = np.isfinite(xy).all(axis=1)
                xy = xy[mask]
                if xy.size > 0:
                    return xy.astype(np.float32, copy=False)
    return None

def stream_and_plot(data_port: str):
    print(f"\n== Streaming on {data_port} @ 921600 ==")

    plt.ion()
    fig, ax = plt.subplots()
    sc = ax.scatter([], [])
    ax.set_xlim(-5, 5)
    ax.set_ylim(0, 10)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("TLV1 Point Cloud (X vs Y)")

    buf = bytearray()

    pkts = 0
    updates = 0
    last = time.time()

    with serial.Serial(data_port, 921600, timeout=0.05) as ser:
        while True:
            plt.pause(0.001)

            chunk = ser.read(4096)
            if not chunk:
                time.sleep(0.002)
                continue
            buf.extend(chunk)

            mi = buf.find(MAGIC)
            if mi == -1:
                if len(buf) > 50000:
                    del buf[:-20000]
                continue

            if len(buf) < mi + 40:
                continue

            header = buf[mi:mi + 40]
            total_len = struct.unpack_from("<I", header, 12)[0]
            if total_len < 40 or total_len > 20000:
                del buf[:mi + 8]
                continue
            if len(buf) < mi + total_len:
                continue

            pkt = bytes(buf[mi:mi + total_len])
            del buf[:mi + total_len]
            pkts += 1

            num_tlvs = struct.unpack_from("<I", pkt, 28)[0]
            offset = 40

            for _ in range(num_tlvs):
                if offset + 8 > len(pkt):
                    break
                tlv_type, tlv_len = struct.unpack_from("<II", pkt, offset)
                offset += 8
                if tlv_len < 8 or offset + (tlv_len - 8) > len(pkt):
                    break

                payload = pkt[offset:offset + tlv_len - 8]

                if tlv_type == 1:
                    xy = parse_tlv1_xy(payload)
                    if xy is not None and xy.size > 0:
                        sc.set_offsets(xy)
                        updates += 1

                offset += (tlv_len - 8)

            now = time.time()
            if now - last > 1.0:
                print(f"pkts/s ~ {pkts} | plot_updates/s ~ {updates}")
                pkts = 0
                updates = 0
                last = now

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", default="COM4")
    ap.add_argument("--data", default="COM5")
    args = ap.parse_args()

    configure_sensor(args.cfg)
    stream_and_plot(args.data)

if __name__ == "__main__":
    main()
