"""Pure helpers shared by WildBridge GroundStation DJI clients."""

from __future__ import annotations

import json
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import Any

DISCOVERY_RESPONSE_PREFIX = "WILDBRIDGE_HERE:"
COMMON_DISCOVERY_HOSTS = tuple(
    list(range(1, 51)) + list(range(100, 121)) + list(range(150, 171)) + list(range(200, 221))
)


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


def parse_discovery_response_tuple(
    payload: bytes | str,
    fallback_ip: str | None = None,
) -> tuple[str, str] | None:
    """Parse a discovery response into the tuple shape used by legacy ROS callers."""
    response = parse_discovery_response(payload, fallback_ip=fallback_ip)
    if response is None:
        return None
    return response.ip_address, response.name


def candidate_subnet_ips(
    local_ips: Iterable[str],
    host_octets: Iterable[int] = COMMON_DISCOVERY_HOSTS,
) -> list[str]:
    """Build direct-probe candidate IPs for each /24 local subnet."""
    candidates: list[str] = []
    for local_ip in local_ips:
        parts = local_ip.split(".")
        if len(parts) != 4:
            continue
        subnet = ".".join(parts[:3])
        candidates.extend(
            f"{subnet}.{host_octet}"
            for host_octet in host_octets
            if f"{subnet}.{host_octet}" != local_ip
        )
    return candidates


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


def parse_telemetry_chunk(
    buffer: str,
    chunk: bytes | str,
    timestamp_factory: Callable[[], str] | None = None,
) -> tuple[str, list[dict[str, Any]]]:
    """Append a socket chunk to a telemetry buffer and parse complete JSON lines."""
    if isinstance(chunk, bytes):
        buffer += chunk.decode("utf-8")
    else:
        buffer += chunk

    telemetry_items = []
    while "\n" in buffer:
        line, buffer = buffer.split("\n", 1)
        telemetry = parse_telemetry_line(line)
        if telemetry is not None:
            if timestamp_factory is not None:
                telemetry["timestamp"] = timestamp_factory()
            telemetry_items.append(telemetry)

    return buffer, telemetry_items
