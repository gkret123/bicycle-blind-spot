"""
Test script for the DRV2605L haptic driver using Adafruit's CircuitPython library.
Plays a series of haptic effects on a connected motor.

Wiring instructions:
    3.3V (Pin 1) → DRV2605L VIN
    GND (Pin 6) → DRV2605L GND
    SDA (GPIO2, Pin 3) → DRV2605L SDA
    SCL (GPIO3, Pin 5) → DRV2605L SCL
    Motor leads → DRV2605L M+ / M-
"""



import time
import board
import busio
import adafruit_drv2605

def play_effect(drv, effect_id, pause=0.2):
    drv.sequence[0] = adafruit_drv2605.Effect(effect_id)
    drv.sequence[1] = adafruit_drv2605.Pause(0)  # end marker
    drv.play()
    time.sleep(pause)

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    drv = adafruit_drv2605.DRV2605(i2c)

    # Use internal trigger mode: DRV plays effects from its internal library
    drv.mode = adafruit_drv2605.MODE_INTTRIG

    print("Testing DRV2605L effects... Ctrl+C to exit")

    # A handful of effects that usually feel distinct on ERM motors
    effects = [
        ("Strong Click", 1),
        ("Sharp Click", 4),
        ("Double Click", 7),
        ("Soft Bump", 10),
        ("Pulse", 15),
        ("Buzz / Rumble-ish", 47),
    ]

    for name, eid in effects:
        print(f"Playing: {name} (Effect {eid})")
        play_effect(drv, eid, pause=0.6)

    print("Done.")

if __name__ == "__main__":
    main()
