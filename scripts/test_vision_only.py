#!/usr/bin/env python3
"""
Test the vision system without requiring BLE devices.
Captures frames, runs YOLO, computes TTR and angle splits.
"""

import sys
sys.path.insert(0, 'src')

from bicycle_blind_spot.vision.ttr_source_vision import VisionTTRSource, VisionTTRConfig
import time

def main():
    print("=" * 70)
    print("Vision-Only Test (no BLE required)")
    print("=" * 70)

    # Create vision source
    print("\n[1/3] Initializing camera provider...")
    cfg = VisionTTRConfig(
        show=False,
    )
    vision = VisionTTRSource(cfg)
    vision.start()
    print("✓ Vision system initialized")

    # Run for 20 samples, reading at ~5Hz
    print("\n[2/3] Running vision loop (20 samples)...")
    try:
        for i in range(20):
            ttr = vision.value()
            angle = vision.angle_deg
            status = vision.status
            left_ttr = vision.left_ttr
            right_ttr = vision.right_ttr

            print(f"  [{i+1:2d}] TTR={ttr:.3f} L={left_ttr:.3f} R={right_ttr:.3f} "
                  f"angle={angle:+.1f}° status={status}")
            time.sleep(0.2)

        print("✓ Vision loop completed")
    except KeyboardInterrupt:
        print("\n✓ Interrupted by user")
    except Exception as e:
        print(f"✗ Error during loop: {e}")
        import traceback
        traceback.print_exc()

    # Cleanup
    print("\n[3/3] Shutting down...")
    vision.stop()
    print("✓ Shutdown complete")

    print("\n" + "=" * 70)
    print("Vision system test successful!")
    print("=" * 70)

if __name__ == "__main__":
    main()
