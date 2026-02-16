import time, struct, argparse
import serial
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"

CMDS = [
    "sensorStop","flushCfg","dfeDataOutputMode 1","channelCfg 15 5 0","adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1","lowPower 0 0",
    "profileCfg 0 77 7 3 39 0 0 100 1 256 7200 0 0 30",
    "chirpCfg 0 0 0 0 0 0 0 1","chirpCfg 1 1 0 0 0 0 0 4",
    "frameCfg 0 1 32 0 100 1 0",
    "guiMonitor -1 1 1 1 0 0 1",
    "cfarCfg -1 0 2 8 4 3 0 15.0 0","cfarCfg -1 1 0 4 2 3 1 15.0 0",
    "multiObjBeamForming -1 1 0.5","calibDcRangeSig -1 0 -5 8 256",
    "clutterRemoval -1 1",
    "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0",
    "measureRangeBiasAndRxChanPhase 0 1. 0.2","aoaFovCfg -1 -90 90 -90 90",
    "cfarFovCfg -1 0 0.25 8.64","cfarFovCfg -1 1 -10.59 10.59","extendedMaxVelocity -1 0",
    "CQRxSatMonitor 0 3 11 121 0","CQSigImgMonitor 0 127 8","analogMonitor 0 0",
    "lvdsStreamCfg -1 0 0 0","calibData 0 0 0","sensorStart",
]

def is_hard_error(line: str) -> bool:
    l = line.lower()
    return ("error" in l) or ("invalid" in l) or ("unknown" in l) or ("not recognized" in l)

def send_cmd_wait_done(ser, cmd, timeout_s):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode()); ser.flush()
    t0 = time.time(); saw_any=False
    while time.time()-t0 < timeout_s:
        line = ser.readline()
        if not line: continue
        s = line.decode(errors="ignore").strip()
        if not s: continue
        saw_any=True
        sl = s.lower()
        if "ignored:" in sl and ("already stopped" in sl or "already started" in sl):
            return True
        if is_hard_error(s):
            print("  ", s)
            return False
        if "Done" in s:
            return True
    if cmd=="sensorStop" and saw_any:
        return True
    return False

def configure_sensor(cfg_port):
    print(f"\n== Configuring sensor on {cfg_port} @ 115200 ==")
    with serial.Serial(cfg_port, 115200, timeout=0.2, write_timeout=1) as s:
        time.sleep(0.25)
        for c in CMDS:
            to = 4.0 if c=="sensorStart" else 1.5
            if not send_cmd_wait_done(s, c, to):
                raise RuntimeError(f"Command failed: {c}")
    print("✅ Sensor started.")

def parse_tlv1_points(payload: bytes):
    L=len(payload)
    for trim in (0,8,4,12):
        if L>trim and ((L-trim)%16)==0:
            body = payload[:L-trim]
            pts = np.frombuffer(body, dtype=np.float32).reshape(-1,4)  # x,y,z,vel
            pts = pts[np.isfinite(pts).all(axis=1)]
            return pts
    return None

def clusters_by_grid(pts, cell=0.6, min_pts=3):
    x = pts[:,0]; y = pts[:,1]
    gx = np.floor(x / cell).astype(int)
    gy = np.floor(y / cell).astype(int)
    keys = gx * 100000 + gy
    uniq, counts = np.unique(keys, return_counts=True)
    out = []
    for k, c in zip(uniq, counts):
        if c < min_pts:
            continue
        m = (keys == k)
        clu = pts[m]
        center = np.array([np.median(clu[:,0]), np.median(clu[:,1])], dtype=np.float32)
        out.append((center, clu))
    return out

def main(cfg="/dev/ttyUSB0", data="/dev/ttyUSB1"):
    configure_sensor(cfg)

    plt.ion()
    fig, ax = plt.subplots()
    sc_all = ax.scatter([], [], s=6, alpha=0.25)
    sc_clu = ax.scatter([], [], s=20, alpha=0.9)
    ax.set_xlim(-8, 8); ax.set_ylim(0, 25)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("Approach test v3 (sticky cluster + easier human thresholds)")

    buf=bytearray()

    # History for the selected (sticky) cluster
    y_hist = deque(maxlen=14)   # ~1.4 s
    v_hist = deque(maxlen=14)

    # Human-friendly thresholds
    v_thresh = 0.35     # require median vel < -0.35
    min_drop_m = 0.25   # range must drop by 0.25 m over window
    need_len = 8        # need at least 8 frames of history

    # ROI
    roi = dict(xmax=6.0, ymin=0.8, ymax=25.0)

    prev_center = None
    last_print = time.time()

    with serial.Serial(data, 921600, timeout=0.05) as ser:
        while True:
            plt.pause(0.001)
            chunk = ser.read(4096)
            if not chunk:
                time.sleep(0.002); continue
            buf.extend(chunk)

            mi = buf.find(MAGIC)
            if mi==-1:
                if len(buf)>50000: del buf[:-20000]
                continue
            if len(buf) < mi+40: continue

            header = buf[mi:mi+40]
            total_len = struct.unpack_from("<I", header, 12)[0]
            if total_len<40 or total_len>20000:
                del buf[:mi+8]; continue
            if len(buf) < mi+total_len: continue

            pkt = bytes(buf[mi:mi+total_len])
            del buf[:mi+total_len]

            num_tlvs = struct.unpack_from("<I", pkt, 28)[0]
            offset=40

            approaching_now = False
            got_cluster = False

            for _ in range(num_tlvs):
                if offset+8>len(pkt): break
                tlv_type, tlv_len = struct.unpack_from("<II", pkt, offset); offset+=8
                if tlv_len<8 or offset+(tlv_len-8)>len(pkt): break
                payload = pkt[offset:offset+tlv_len-8]

                if tlv_type==1:
                    pts = parse_tlv1_points(payload)
                    if pts is not None and pts.size>0:
                        x,y,vel = pts[:,0], pts[:,1], pts[:,3]
                        m = (np.abs(x) < roi["xmax"]) & (y > roi["ymin"]) & (y < roi["ymax"])
                        pts2 = pts[m]
                        sc_all.set_offsets(pts2[:, :2] if pts2.size else np.empty((0,2)))

                        clus = clusters_by_grid(pts2, cell=0.6, min_pts=3)
                        if clus:
                            # choose cluster closest to previous center (sticky), else nearest (smallest y)
                            if prev_center is not None:
                                dists = [np.linalg.norm(c[0]-prev_center) for c in clus]
                                idx = int(np.argmin(dists))
                            else:
                                ys = [c[0][1] for c in clus]
                                idx = int(np.argmin(ys))

                            center, clu = clus[idx]
                            prev_center = center
                            got_cluster = True

                            sc_clu.set_offsets(clu[:, :2])

                            y_med = float(np.median(clu[:,1]))
                            v_med = float(np.median(clu[:,3]))
                            y_hist.append(y_med)
                            v_hist.append(v_med)

                            if len(y_hist) >= need_len:
                                drop = y_hist[0] - y_hist[-1]
                                v_ok = (np.median(v_hist) < -v_thresh)
                                approaching_now = v_ok and (drop > min_drop_m)
                        else:
                            sc_clu.set_offsets(np.empty((0,2)))

                offset += (tlv_len-8)

            # decay if cluster lost
            if not got_cluster:
                prev_center = None
                if len(y_hist) > 0: y_hist.popleft()
                if len(v_hist) > 0: v_hist.popleft()

            now = time.time()
            if now - last_print > 0.5:
                if len(y_hist) >= 2:
                    drop = y_hist[0] - y_hist[-1]
                    vmed = float(np.median(v_hist)) if v_hist else float("nan")
                    print(f"approaching_now={approaching_now}  y_drop={drop:.2f}m  v_med={vmed:.2f} m/s  hist_len={len(y_hist)}")
                else:
                    print(f"approaching_now={approaching_now}  hist_len={len(y_hist)}")
                last_print = now

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", default="/dev/ttyUSB0")
    ap.add_argument("--data", default="/dev/ttyUSB1")
    a = ap.parse_args()
    main(a.cfg, a.data)
