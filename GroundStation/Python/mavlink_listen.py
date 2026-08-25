"""Listen to a WildBridge MAVLink 2 telemetry endpoint and print what arrives.

A field check for the MAVLink telemetry phase: confirms the aircraft is emitting well-formed
MAVLink 2 before anyone opens QGroundControl, and shows which messages are actually arriving and
at what rate. Read-only — it never transmits, so it cannot influence the aircraft.

Usage::

    python mavlink_listen.py                 # listen on 0.0.0.0:14550
    python mavlink_listen.py --port 14550    # same, explicit
    python mavlink_listen.py --summary 5     # print a rate summary every 5 seconds

Requires pymavlink (``pip install pymavlink``). It is not a dependency of the rest of the
GroundStation code, so this script degrades with a clear message rather than a traceback.
"""

from __future__ import annotations

import argparse
import collections
import sys
import time
from typing import Any

DEFAULT_PORT = 14550
DEFAULT_SUMMARY_INTERVAL_S = 5.0

# Messages the telemetry phase emits, in the order most useful to see first.
EXPECTED_MESSAGES = (
    "HEARTBEAT",
    "SYS_STATUS",
    "GPS_RAW_INT",
    "ATTITUDE",
    "GLOBAL_POSITION_INT",
    "VFR_HUD",
    "BATTERY_STATUS",
    "HOME_POSITION",
    "AUTOPILOT_VERSION",
    "STATUSTEXT",
)


def _load_mavlink() -> Any:
    try:
        from pymavlink import mavutil
    except ImportError:
        print(
            "pymavlink is not installed. Install it with:\n    pip install pymavlink",
            file=sys.stderr,
        )
        raise SystemExit(1) from None
    return mavutil


def describe(message: Any) -> str:
    """One-line summary of the messages worth reading at a glance."""
    kind = message.get_type()
    if kind == "HEARTBEAT":
        return (
            f"sysid={message.get_srcSystem()} custom_mode={message.custom_mode} "
            f"base_mode=0x{message.base_mode:02X} state={message.system_status} "
            f"autopilot={message.autopilot} type={message.type}"
        )
    if kind == "GLOBAL_POSITION_INT":
        return (
            f"lat={message.lat / 1e7:.7f} lon={message.lon / 1e7:.7f} "
            f"amsl={message.alt / 1000:.1f}m agl={message.relative_alt / 1000:.1f}m "
            f"hdg={message.hdg / 100:.1f}deg"
        )
    if kind == "GPS_RAW_INT":
        return f"fix={message.fix_type} sats={message.satellites_visible}"
    if kind == "ATTITUDE":
        return f"roll={message.roll:.3f} pitch={message.pitch:.3f} yaw={message.yaw:.3f} (rad)"
    if kind == "VFR_HUD":
        return (
            f"groundspeed={message.groundspeed:.1f}m/s alt={message.alt:.1f}m "
            f"climb={message.climb:.1f}m/s heading={message.heading}deg"
        )
    if kind == "BATTERY_STATUS":
        remaining = getattr(message, "time_remaining", None)
        return f"remaining={message.battery_remaining}% time_remaining={remaining}s"
    if kind == "HOME_POSITION":
        return f"lat={message.latitude / 1e7:.7f} lon={message.longitude / 1e7:.7f}"
    if kind == "STATUSTEXT":
        return f"[{message.severity}] {message.text}"
    if kind == "AUTOPILOT_VERSION":
        return f"capabilities=0x{message.capabilities:X}"
    return ""


def print_summary(counts: collections.Counter, elapsed_s: float) -> None:
    print(f"\n--- {elapsed_s:.1f}s ---")
    for name in EXPECTED_MESSAGES:
        count = counts.get(name, 0)
        rate = count / elapsed_s if elapsed_s > 0 else 0.0
        mark = " " if count else "!"
        print(f" {mark} {name:22} {count:6}  {rate:6.2f} Hz")
    unexpected = sorted(set(counts) - set(EXPECTED_MESSAGES))
    for name in unexpected:
        print(f"   {name:22} {counts[name]:6}  (not part of the telemetry phase)")
    print()


def listen(port: int, summary_interval_s: float) -> None:
    mavutil = _load_mavlink()
    connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}", dialect="common")
    print(f"Listening for MAVLink 2 on udp:0.0.0.0:{port} (Ctrl-C to stop)")

    counts: collections.Counter = collections.Counter()
    seen: set[str] = set()
    started = time.monotonic()
    last_summary = started

    while True:
        message = connection.recv_match(blocking=True, timeout=1.0)
        now = time.monotonic()

        if message is not None:
            kind = message.get_type()
            if kind == "BAD_DATA":
                # A malformed frame is the whole point of running this: report it loudly rather
                # than silently dropping it, because it means the sender and this dialect disagree.
                print(f"BAD_DATA: {message.reason if hasattr(message, 'reason') else message}")
            else:
                counts[kind] += 1
                if kind not in seen:
                    seen.add(kind)
                    detail = describe(message)
                    print(f"first {kind:22} {detail}")

        if summary_interval_s > 0 and now - last_summary >= summary_interval_s:
            print_summary(counts, now - started)
            last_summary = now


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port to listen on")
    parser.add_argument(
        "--summary",
        type=float,
        default=DEFAULT_SUMMARY_INTERVAL_S,
        dest="summary_interval_s",
        help="seconds between rate summaries, 0 to disable",
    )
    args = parser.parse_args()

    try:
        listen(args.port, args.summary_interval_s)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
