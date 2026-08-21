"""Pure event and discovery helpers for the video test webapp."""

from __future__ import annotations

import json
from collections.abc import Callable
from typing import Any

DISCOVERY_RESPONSE_PREFIX = "WILDBRIDGE_HERE:"


def build_event_entry(
    event_type: str,
    payload: dict[str, Any],
    timestamp_factory: Callable[[], str],
) -> dict[str, Any]:
    """Build one event-log entry with stable core fields."""
    return {"ts": timestamp_factory(), "type": event_type, **payload}


def serialize_ndjson_entry(entry: dict[str, Any]) -> str:
    """Serialize one event-log entry as compact NDJSON."""
    return json.dumps(entry, separators=(",", ":")) + "\n"


def format_sse_message(message_type: str, payload: Any) -> bytes:
    """Serialize one server-sent event message."""
    return f"data: {json.dumps({'type': message_type, 'payload': payload})}\n\n".encode()


def parse_discovery_response(message: str, remote_ip: str) -> dict[str, str] | None:
    """Parse a WildBridge discovery response for the video grid."""
    if not message.startswith(DISCOVERY_RESPONSE_PREFIX):
        return None
    payload = message[len(DISCOVERY_RESPONSE_PREFIX) :]
    if ":" not in payload:
        return {"ip": remote_ip, "name": payload or remote_ip}
    ip_address, name = payload.split(":", 1)
    return {"ip": ip_address or remote_ip, "name": name.strip() or remote_ip}
