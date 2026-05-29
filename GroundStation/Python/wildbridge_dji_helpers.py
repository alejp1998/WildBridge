"""Small pure helpers shared by WildBridge GroundStation DJI clients."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

DISCOVERY_RESPONSE_PREFIX = "WILDBRIDGE_HERE:"


@dataclass(frozen=True)
class DiscoveryResponse:
    ip_address: str
    name: str = "UNKNOWN"


def parse_discovery_response(
    payload: bytes | str,
    fallback_ip: str | None = None,
) -> DiscoveryResponse | None:
    """Parse a WildBridge UDP discovery response."""
    if isinstance(payload, bytes):
        try:
            message = payload.decode("utf-8")
        except UnicodeDecodeError:
            return None
    else:
        message = payload

    message = message.strip()
    if not message.startswith(DISCOVERY_RESPONSE_PREFIX):
        return None

    parts = message.split(":")
    ip_address = parts[1].strip() if len(parts) > 1 else ""
    if not ip_address and fallback_ip:
        ip_address = fallback_ip
    if not ip_address:
        return None

    name = parts[2].strip() if len(parts) > 2 and parts[2].strip() else "UNKNOWN"
    return DiscoveryResponse(ip_address=ip_address, name=name)


def build_command_url(base_url: str, endpoint: str) -> str:
    """Join a drone command base URL and endpoint without duplicate slashes."""
    if not endpoint:
        return base_url.rstrip("/")
    return f"{base_url.rstrip('/')}/{endpoint.lstrip('/')}"


def parse_telemetry_line(
    line: str,
    timestamp: str | None = None,
) -> dict[str, Any] | None:
    """Parse one newline-delimited telemetry JSON object."""
    stripped_line = line.strip()
    if not stripped_line:
        return None

    try:
        telemetry = json.loads(stripped_line)
    except json.JSONDecodeError:
        return None

    if not isinstance(telemetry, dict):
        return None

    if timestamp is not None:
        telemetry["timestamp"] = timestamp
    return telemetry
