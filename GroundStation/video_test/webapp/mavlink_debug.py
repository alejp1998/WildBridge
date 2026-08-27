"""MAVLink debugging surface for the webapp.

Exists because of how this project's MAVLink defects have actually been found. None of them came
from reading code: they came from reading the same aircraft over both wires at the same moment and
looking at where the two disagreed. That comparison caught a camera heartbeat overwriting the
flight mode, a gimbal message with two fields transposed, a home-position gate on the wrong latch,
and a heading reported in the wrong convention.

This makes that comparison a permanent instrument rather than a script someone rewrites each time.
It samples both transports against one drone and reports, field by field, what each wire says and
whether they agree.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from wildbridge_groundstation.dji_client import DJIInterface
from wildbridge_groundstation.transport import Transport

#: How long a sampler stays alive after the last request for it. A debugging page that is closed
#: should not leave a socket bound to the aircraft's telemetry port forever.
IDLE_TIMEOUT_S = 120.0

#: Fields where a small numeric difference is sampling noise rather than disagreement. Both wires
#: are read a fraction of a second apart, and the aircraft does not stand perfectly still.
NUMERIC_TOLERANCE = 1.0


class _Comparison:
    """One drone read over both wires at once."""

    def __init__(self, ip: str, mavlink_port: int, peer_port: int):
        self.ip = ip
        self.touched = time.monotonic()
        self._lock = threading.Lock()
        self._http = DJIInterface(ip, transport=Transport.HTTP)
        self._mavlink = DJIInterface(
            ip,
            transport=Transport.MAVLINK,
            mavlink_port=mavlink_port,
            mavlink_peer_port=peer_port,
        )
        self._http.startTelemetryStream()
        self._mavlink.startTelemetryStream()

    def snapshot(self) -> dict[str, Any]:
        self.touched = time.monotonic()
        with self._lock:
            http = self._http.getTelemetry()
            mavlink = self._mavlink.getTelemetry()
        return {
            "ip": self.ip,
            "httpKeys": len(http),
            "mavlinkKeys": len([k for k in mavlink if not k.startswith("_")]),
            "rows": _rows(http, mavlink),
        }

    def close(self) -> None:
        self._http.stopTelemetryStream()
        self._mavlink.stopTelemetryStream()


def _rows(http: dict[str, Any], mavlink: dict[str, Any]) -> list[dict[str, Any]]:
    """One row per field, ordered so disagreements come first."""
    rows = []
    for key in sorted(set(http) | set(mavlink)):
        if key.startswith("_"):
            # Working state the transport keeps for itself, not telemetry.
            continue
        on_http = key in http
        on_mavlink = key in mavlink
        rows.append(
            {
                "key": key,
                "http": _render(http.get(key)),
                "mavlink": _render(mavlink.get(key)),
                "status": _status(http.get(key), mavlink.get(key), on_http, on_mavlink),
            }
        )
    order = {"differ": 0, "mavlinkOnly": 1, "httpOnly": 2, "agree": 3}
    rows.sort(key=lambda r: (order.get(r["status"], 9), r["key"]))
    return rows


def _status(http_value, mavlink_value, on_http: bool, on_mavlink: bool) -> str:
    if not on_mavlink:
        return "httpOnly"
    if not on_http:
        return "mavlinkOnly"
    return "agree" if _close_enough(http_value, mavlink_value) else "differ"


def _close_enough(a: Any, b: Any) -> bool:
    if a == b:
        return True
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) <= NUMERIC_TOLERANCE
    if isinstance(a, dict) and isinstance(b, dict):
        keys = set(a) | set(b)
        return all(_close_enough(a.get(k), b.get(k)) for k in keys)
    return False


def _render(value: Any) -> str:
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:.4g}"
    if isinstance(value, dict):
        return ", ".join(f"{k}={_render(v)}" for k, v in sorted(value.items()))
    return str(value)


class ComparisonRegistry:
    """Keeps one sampler per drone, and reaps them when the page stops asking."""

    def __init__(self, mavlink_port: int, peer_port: int):
        self._mavlink_port = mavlink_port
        self._peer_port = peer_port
        self._lock = threading.Lock()
        self._samplers: dict[str, _Comparison] = {}

    def snapshot(self, name: str, ip: str) -> dict[str, Any]:
        with self._lock:
            self._reap()
            sampler = self._samplers.get(name)
            if sampler is not None and sampler.ip != ip:
                # The drone moved networks; the old sampler is bound to an address that is no
                # longer it, and would sit reporting nothing forever.
                sampler.close()
                sampler = None
            if sampler is None:
                sampler = _Comparison(ip, self._mavlink_port, self._peer_port)
                self._samplers[name] = sampler
        return sampler.snapshot()

    def _reap(self) -> None:
        now = time.monotonic()
        for name, sampler in list(self._samplers.items()):
            if now - sampler.touched > IDLE_TIMEOUT_S:
                sampler.close()
                del self._samplers[name]

    def close_all(self) -> None:
        with self._lock:
            for sampler in self._samplers.values():
                sampler.close()
            self._samplers.clear()
