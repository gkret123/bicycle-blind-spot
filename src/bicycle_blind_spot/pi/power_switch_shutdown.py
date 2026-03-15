#!/usr/bin/env python3
"""Monitor a GPIO power switch and trigger a graceful system shutdown."""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time

from gpiozero import Button


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Shutdown monitor for a Raspberry Pi power switch."
    )
    parser.add_argument(
        "--pin",
        type=int,
        default=17,
        help="BCM GPIO pin connected to the OFF switch signal (default: 17).",
    )
    parser.add_argument(
        "--bounce-time",
        type=float,
        default=0.2,
        help="Debounce interval for switch events in seconds (default: 0.2).",
    )
    parser.add_argument(
        "--hold-time",
        type=float,
        default=0.8,
        help="How long the switch must remain active before shutdown in seconds (default: 0.8).",
    )
    parser.add_argument(
        "--active-state",
        choices=("low", "high"),
        default="low",
        help="Switch electrical active state (default: low, for pull-up wiring).",
    )
    parser.add_argument(
        "--app-service",
        default="bicycle-blind-spot.service",
        help="systemd service name for the main app (default: bicycle-blind-spot.service).",
    )
    return parser.parse_args()


def run_command(cmd: list[str]) -> None:
    print(f"Running: {' '.join(cmd)}", flush=True)
    subprocess.run(cmd, check=False)


def main() -> int:
    args = parse_args()

    active_low = args.active_state == "low"
    button = Button(
        args.pin,
        pull_up=active_low,
        bounce_time=args.bounce_time,
        hold_time=args.hold_time,
    )

    stop_requested = False

    def request_exit(_signum: int, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, request_exit)
    signal.signal(signal.SIGINT, request_exit)

    print(
        f"Watching GPIO{args.pin} for shutdown switch (active_{'low' if active_low else 'high'}).",
        flush=True,
    )

    while not stop_requested:
        button.wait_for_active(timeout=0.25)
        if stop_requested:
            break
        if button.is_active:
            print("Power switch turned off. Stopping app service and shutting down.", flush=True)
            run_command(["systemctl", "stop", args.app_service])
            run_command(["shutdown", "-h", "now"])
            time.sleep(2)

    print("Shutdown monitor exiting.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())