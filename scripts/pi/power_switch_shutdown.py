#!/usr/bin/env python3
"""Compatibility wrapper for the packaged shutdown monitor."""

from bicycle_blind_spot.pi.power_switch_shutdown import main

if __name__ == "__main__":
    raise SystemExit(main())