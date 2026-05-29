from wildbridge_dji_helpers import (
    DiscoveryResponse,
    build_command_url,
    parse_discovery_response,
    parse_telemetry_line,
)


def test_parse_discovery_response_accepts_ip_only_payload():
    assert parse_discovery_response(b"WILDBRIDGE_HERE:192.168.1.42") == DiscoveryResponse(
        ip_address="192.168.1.42",
        name="UNKNOWN",
    )


def test_parse_discovery_response_accepts_named_payload():
    assert parse_discovery_response("WILDBRIDGE_HERE:192.168.1.42:mini1") == DiscoveryResponse(
        ip_address="192.168.1.42",
        name="mini1",
    )


def test_parse_discovery_response_uses_fallback_ip_when_payload_ip_is_empty():
    assert parse_discovery_response(
        "WILDBRIDGE_HERE::mini1", fallback_ip="192.168.1.9"
    ) == DiscoveryResponse(
        ip_address="192.168.1.9",
        name="mini1",
    )


def test_parse_discovery_response_rejects_invalid_payloads():
    assert parse_discovery_response("OTHER:192.168.1.42") is None
    assert parse_discovery_response(b"\xff") is None
    assert parse_discovery_response("WILDBRIDGE_HERE:") is None


def test_build_command_url_normalizes_slashes():
    assert build_command_url("http://192.168.1.42:8080", "/send/takeoff") == (
        "http://192.168.1.42:8080/send/takeoff"
    )
    assert build_command_url("http://192.168.1.42:8080/", "send/takeoff") == (
        "http://192.168.1.42:8080/send/takeoff"
    )


def test_parse_telemetry_line_adds_timestamp_to_json_object():
    assert parse_telemetry_line(
        '{"heading": 42.5, "speed": {"x": 1.0}}',
        timestamp="2026-05-29_12-00-00.000000",
    ) == {
        "heading": 42.5,
        "speed": {"x": 1.0},
        "timestamp": "2026-05-29_12-00-00.000000",
    }


def test_parse_telemetry_line_rejects_empty_invalid_and_non_object_lines():
    assert parse_telemetry_line("") is None
    assert parse_telemetry_line("not-json") is None
    assert parse_telemetry_line("[1, 2, 3]") is None
