"""
DRV2605L TTR-scaling demo (Amplitude scaling vs Pulse-rate scaling)

Requires:
  - Adafruit Blinka
  - adafruit-circuitpython-drv2605

Wiring:
  3.3V  -> VIN
  GND   -> GND
  SDA   -> SDA
  SCL   -> SCL
  Motor -> M+ / M-
"""

import time
import board
import busio
import adafruit_drv2605



MOTOR_TYPE = "ERM"          # "ERM" (default) or "LRA" (uses drv.use_LRM())
TTR_START = 10.0            # seconds: starts "safe", ramps down to 0

# Demo 1: amplitude scaling (fixed pulse rate, variable amplitude)
AMP_MIN = 20                # floor so you still feel it (0 = off)
AMP_MAX = 127               # typical max for positive RTP values
AMP_PULSE_ON = 0.12         # seconds on-time for each pulse
AMP_PERIOD = 0.60           # seconds between pulse starts (fixed rate)

# Demo 2: pulse-rate scaling (fixed amplitude, variable interval)
RATE_AMP = 127               # constant MAX amplitude for pulses
RATE_PULSE_ON = 0.12        # seconds on-time for each pulse
INTERVAL_SLOW = 1.20        # seconds between pulses at low threat (high TTR)
INTERVAL_FAST = 0.18        # seconds between pulses at high threat (low TTR)

# If you want the pulse-rate demo to use the DRV’s internal effect library instead
# of real-time pulses, set this True:
USE_INTERNAL_EFFECT_FOR_RATE_DEMO = False
EFFECT_ID_FOR_RATE_DEMO = 47  # only used if USE_INTERNAL_EFFECT_FOR_RATE_DEMO = True


# -----------------------------
# Helpers
# -----------------------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def ttr_to_scalar(ttr_seconds: float) -> float:
    #Inverted & scaled factor: TTR down => scalar up (0..1)
    if TTR_START <= 0:
        return 1.0
    return clamp(1.0 - (ttr_seconds / TTR_START), 0.0, 1.0)


def curve(s: float) -> float:
    #try s**1.0 (linear), s**3.0 (more aggressive), etc.
    return s * s


def init_drv():
    i2c = busio.I2C(board.SCL, board.SDA)
    drv = adafruit_drv2605.DRV2605(i2c)

    # Motor type configuration (library default is ERM; LRA uses use_LRM()).
    # :contentReference[oaicite:1]{index=1}
    if MOTOR_TYPE.upper() == "LRA":
        drv.use_LRM()
    else:
        drv.use_ERM()

    # Start in internal trigger mode (safe default).
    drv.mode = adafruit_drv2605.MODE_INTTRIG
    return drv


# -----------------------------
# Demo 1: Amplitude scaling
# -----------------------------
def demo_amplitude_scaling(drv):
    print("\n=== Demo 1: Amplitude scaling (fixed pulse rate) ===")
    print("TTR decreases -> amplitude increases (Real-Time Playback mode).")
    # Real-time playback: drive motor continuously with realtime_value. :contentReference[oaicite:2]{index=2}
    drv.realtime_value = 0
    drv.mode = adafruit_drv2605.MODE_REALTIME

    start = time.monotonic()
    next_pulse_t = start
    last_print_sec = -1

    while True:
        now = time.monotonic()
        elapsed = now - start
        ttr = max(0.0, TTR_START - elapsed)
        s = ttr_to_scalar(ttr)
        sc = curve(s)

        amp = int(AMP_MIN + sc * (AMP_MAX - AMP_MIN))
        amp = clamp(amp, 0, 127)

        # Pulse at a fixed rate; only amplitude changes
        if now >= next_pulse_t:
            drv.realtime_value = amp
            time.sleep(AMP_PULSE_ON)
            drv.realtime_value = 0
            next_pulse_t += AMP_PERIOD

        # Print once per second
        sec = int(elapsed)
        if sec != last_print_sec:
            last_print_sec = sec
            print(f"TTR={ttr:5.2f}s  scalar={s:4.2f}  amp={amp:3d}")

        if ttr <= 0.0:
            break

        time.sleep(0.005)

    drv.realtime_value = 0
    drv.mode = adafruit_drv2605.MODE_INTTRIG


# -----------------------------
# Demo 2: Pulse-rate scaling
# -----------------------------
def demo_pulse_rate_scaling(drv):
    print("\n=== Demo 2: Pulse-rate scaling (fixed amplitude) ===")
    print("TTR decreases -> pulses come faster (interval shrinks).")

    start = time.monotonic()
    last_print_sec = -1

    if USE_INTERNAL_EFFECT_FOR_RATE_DEMO:
        # Use internal waveforms (more “designed” feel), but intensity is mostly via timing here.
        drv.mode = adafruit_drv2605.MODE_INTTRIG
        drv.sequence[0] = adafruit_drv2605.Effect(EFFECT_ID_FOR_RATE_DEMO)
        drv.sequence[1] = adafruit_drv2605.Effect(0)  # end marker

    else:
        # Use real-time pulses for consistent “fixed amplitude” control.
        drv.realtime_value = 0
        drv.mode = adafruit_drv2605.MODE_REALTIME

    while True:
        now = time.monotonic()
        elapsed = now - start
        ttr = max(0.0, TTR_START - elapsed)
        s = ttr_to_scalar(ttr)
        sc = curve(s)

        interval = INTERVAL_SLOW - sc * (INTERVAL_SLOW - INTERVAL_FAST)
        interval = clamp(interval, INTERVAL_FAST, INTERVAL_SLOW)

        # One pulse, then wait (interval controls “urgency”)
        if USE_INTERNAL_EFFECT_FOR_RATE_DEMO:
            drv.play()
            time.sleep(interval)
        else:
            drv.realtime_value = int(RATE_AMP)
            time.sleep(RATE_PULSE_ON)
            drv.realtime_value = 0
            time.sleep(max(0.0, interval - RATE_PULSE_ON))

        # Print once per second
        sec = int(elapsed)
        if sec != last_print_sec:
            last_print_sec = sec
            print(f"TTR={ttr:5.2f}s  scalar={s:4.2f}  interval={interval:4.2f}s")

        if ttr <= 0.0:
            break

    # Stop cleanly
    if USE_INTERNAL_EFFECT_FOR_RATE_DEMO:
        drv.stop()
    else:
        drv.realtime_value = 0
        drv.mode = adafruit_drv2605.MODE_INTTRIG


def main():
    drv = init_drv()
    print("Starting TTR scaling demos... Ctrl+C to exit.\n")

    try:
        demo_amplitude_scaling(drv)
        time.sleep(1.0)
        demo_pulse_rate_scaling(drv)
        print("\nDone.")
    except KeyboardInterrupt:
        print("\nInterrupted. Stopping motor...")
    finally:
        # Best-effort stop in both modes
        try:
            drv.realtime_value = 0
        except Exception:
            pass
        try:
            drv.stop()
        except Exception:
            pass
        try:
            drv.mode = adafruit_drv2605.MODE_INTTRIG
        except Exception:
            pass


if __name__ == "__main__":
    main()
