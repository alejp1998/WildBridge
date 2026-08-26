"""Transport selection, and the MAVLink translation that stands in for the HTTP wire."""

import math
import struct
import time

import pytest
from wildbridge_groundstation.transport import (
    _COMMAND_MAP,
    _SPECIAL_SENDERS,
    CMD_DO_GRIPPER,
    CMD_DO_REPOSITION,
    CMD_DO_SET_MODE,
    CMD_NAV_TAKEOFF,
    CMD_USER_1,
    CMD_USER_2,
    COMP_ID_AUTOPILOT1,
    GRIPPER_ACTION_RELEASE,
    MAV_RESULT_IN_PROGRESS,
    PX4_MODE_OFFBOARD,
    PX4_MODE_POSCTL,
    UNSUPPORTED_PREFIX,
    USER1_GIMBAL_RELATIVE,
    USER1_RELEASE_MANUAL_OVERRIDE,
    USER2_CAPTURE_TEMPERATURE,
    USER2_LRF_MEASURE,
    WB_FLAG_LRF_TARGET_VALID,
    WB_FLAG_MANUAL_OVERRIDE,
    WB_FLAG_READY_TO_TAKEOFF,
    WILDBRIDGE_STATUS_ID,
    WILDBRIDGE_STATUS_SIZE,
    WILDBRIDGE_STATUS_STRUCT,
    MavlinkCommandChannel,
    Transport,
    _derive,
    _dji_heading,
    apply_mavlink_message,
    decode_wildbridge_status,
    flight_mode_name,
)


class FakeMessage:
    """A decoded MAVLink message, with the header a real one carries."""

    def __init__(self, kind, src_component=COMP_ID_AUTOPILOT1, **fields):
        self._kind = kind
        self._src_component = src_component
        self.__dict__.update(fields)

    def get_type(self):
        return self._kind

    def get_header(self):
        return type("Header", (), {"srcComponent": self._src_component})()


# -- selection --------------------------------------------------------------------------------


def test_default_transport_is_http_so_nothing_changes_by_accident():
    assert Transport.from_env({}) is Transport.HTTP
    assert Transport.from_env({"WB_TRANSPORT": ""}) is Transport.HTTP


@pytest.mark.parametrize(
    ("value", "expected"),
    [("mavlink", Transport.MAVLINK), ("both", Transport.BOTH), ("HTTP", Transport.HTTP)],
)
def test_transport_is_read_from_the_environment(value, expected):
    assert Transport.from_env({"WB_TRANSPORT": value}) is expected


def test_a_misspelled_transport_fails_rather_than_silently_using_http():
    # A typo that quietly ran over HTTP would produce a passing "MAVLink" test result.
    with pytest.raises(ValueError, match="WB_TRANSPORT"):
        Transport.from_env({"WB_TRANSPORT": "mavlnk"})


def test_only_mavlink_only_mode_refuses_to_fall_back():
    assert Transport.MAVLINK.uses_mavlink and not Transport.MAVLINK.allows_http_fallback
    assert Transport.BOTH.uses_mavlink and Transport.BOTH.allows_http_fallback
    assert not Transport.HTTP.uses_mavlink and Transport.HTTP.allows_http_fallback


# -- telemetry translation --------------------------------------------------------------------


def test_heading_is_converted_to_djis_signed_convention():
    # MAVLink reports 0..360, DJI reports -180..180. The same bearing must read the same on
    # either wire, or a consumer sees a 360-degree jump when the transport changes.
    assert _dji_heading(0) == 0.0
    assert _dji_heading(9000) == 90.0
    assert _dji_heading(18000) == 180.0
    assert _dji_heading(19540) == pytest.approx(-164.6)
    assert _dji_heading(27000) == -90.0
    assert _dji_heading(65535) == 0.0, "65535 is MAVLink's unknown"


def test_only_the_autopilots_heartbeat_sets_the_flight_mode():
    telemetry = {}
    apply_mavlink_message(telemetry, FakeMessage("HEARTBEAT", custom_mode=3 << 16))
    assert telemetry["flightMode"] == "GPS_NORMAL"

    # A WildBridge aircraft also heartbeats from its camera component, with a meaningless zero
    # custom_mode. Believing it reports UNKNOWN for a drone sitting in position hold.
    changed = apply_mavlink_message(
        telemetry, FakeMessage("HEARTBEAT", src_component=100, custom_mode=0)
    )
    assert not changed
    assert telemetry["flightMode"] == "GPS_NORMAL"


def test_unadvertised_modes_read_as_unknown():
    assert flight_mode_name(0) == "UNKNOWN"
    assert flight_mode_name(99 << 16) == "UNKNOWN"


def test_position_fills_the_same_keys_the_http_stream_uses():
    telemetry = {}
    apply_mavlink_message(
        telemetry,
        FakeMessage(
            "GLOBAL_POSITION_INT",
            lat=465180000,
            lon=65660000,
            relative_alt=30000,
            hdg=19540,
            vx=100,
            vy=-50,
            vz=25,
        ),
    )
    assert telemetry["location"] == {
        "latitude": pytest.approx(46.518),
        "longitude": pytest.approx(6.566),
        "altitude": pytest.approx(30.0),
    }
    assert telemetry["altitude"] == pytest.approx(30.0)
    assert telemetry["heading"] == pytest.approx(-164.6)
    assert telemetry["speed"] == {"x": 1.0, "y": -0.5, "z": 0.25}


def test_attitude_is_reported_in_degrees_like_the_http_stream():
    telemetry = {}
    apply_mavlink_message(
        telemetry, FakeMessage("ATTITUDE", roll=0.0, pitch=math.radians(-11.3), yaw=math.pi)
    )
    assert telemetry["attitude"]["pitch"] == pytest.approx(-11.3)
    assert telemetry["attitude"]["yaw"] == pytest.approx(180.0)


def test_home_position_is_what_marks_home_as_set():
    telemetry = {}
    apply_mavlink_message(
        telemetry,
        FakeMessage("HOME_POSITION", latitude=465180000, longitude=65660000, altitude=400000),
    )
    assert telemetry["homeSet"] is True
    assert telemetry["homeLocation"]["latitude"] == pytest.approx(46.518)


def test_mission_item_reached_feeds_the_same_latch_the_http_surface_exposes():
    telemetry = {}
    apply_mavlink_message(telemetry, FakeMessage("MISSION_ITEM_REACHED", seq=3))
    assert telemetry["waypointReached"] is True
    assert telemetry["waypointSeq"] == 3


def test_an_unmapped_message_changes_nothing():
    telemetry = {"heading": 12.0}
    assert not apply_mavlink_message(telemetry, FakeMessage("PING"))
    assert telemetry == {"heading": 12.0}


# -- command translation ----------------------------------------------------------------------


def _decode(frame):
    from pymavlink.dialects.v20 import common as mavlink_common

    parser = mavlink_common.MAVLink(None)
    parser.robust_parsing = True
    return (parser.parse_buffer(frame) or [None])[0]


def test_a_goto_is_sent_as_command_int_so_the_position_survives():
    channel = MavlinkCommandChannel("127.0.0.1")
    command, params = _COMMAND_MAP["/send/gotoWaypointHoldHeading"]("46.5180001,6.5660007,30,90,5")
    decoded = _decode(channel._frame_command(command, params))

    assert decoded.get_type() == "COMMAND_INT", "COMMAND_LONG would round lat/lon through float32"
    assert decoded.command == CMD_DO_REPOSITION
    assert decoded.x == 465180001
    assert decoded.y == 65660007
    assert decoded.frame == 6, "altitudes are above home, as everywhere else in WildBridge"
    assert decoded.param4 == pytest.approx(90.0)


def test_nose_forward_is_expressed_as_a_nan_heading():
    channel = MavlinkCommandChannel("127.0.0.1")
    command, params = _COMMAND_MAP["/send/gotoWaypointNoseForward"]("46.518,6.566,30,0,5")
    decoded = _decode(channel._frame_command(command, params))
    assert math.isnan(decoded.param4), "a zero here would silently mean 'hold north'"


def test_a_takeoff_carries_its_altitude_in_param7():
    channel = MavlinkCommandChannel("127.0.0.1")
    command, params = _COMMAND_MAP["/send/takeoff"]("15")
    decoded = _decode(channel._frame_command(command, params))
    assert decoded.get_type() == "COMMAND_LONG"
    assert decoded.command == CMD_NAV_TAKEOFF
    assert decoded.param7 == pytest.approx(15.0)


def test_an_endpoint_with_no_mavlink_equivalent_is_refused_not_guessed():
    channel = MavlinkCommandChannel("127.0.0.1")
    assert not channel.supports("/send/setDroneName")
    assert channel.send("/send/setDroneName", "x").startswith(UNSUPPORTED_PREFIX)


def test_every_command_the_ros_controller_uses_has_a_mavlink_form():
    """Swarm-Steward drives WildBridge through these; a gap means it cannot run on MAVLink.

    Pinned as a test because the failure is silent in the other direction: an endpoint added to
    the ROS controller without a mapping here would simply be refused at flight time.
    """
    required = {
        "/send/takeoff",
        "/send/land",
        "/send/RTH",
        "/send/gotoWaypointNoseForward",
        "/send/gotoWaypointHoldHeading",
        "/send/gotoYaw",
        "/send/gotoAltitude",
        "/send/camera/zoom",
        "/send/camera/startRecording",
        "/send/camera/stopRecording",
        "/send/capture",
        "/send/gimbal/pitch",
        "/send/gimbal/yaw",
        "/send/gimbal/rel_pitch",
        "/send/gimbal/rel_yaw",
        "/send/stick",
        "/send/navigateTrajectoryDJINative",
        "/send/abortMission",
        "/send/abortAll",
        "/send/abort/DJIMission",
        "/send/enableVirtualStick",
        "/send/deactivateManualOverride",
        "/send/setRTHAltitude",
        "/send/setSetting",
        "/send/lrf/measure",
        "/send/captureTemperature",
        "/send/drop",
    }
    covered = set(_COMMAND_MAP) | set(_SPECIAL_SENDERS)
    assert not (required - covered), f"unmapped: {sorted(required - covered)}"


@pytest.mark.parametrize(
    "endpoint", ["/send/abortMission", "/send/abortAll", "/send/abort/DJIMission"]
)
def test_every_abort_is_a_request_for_position_hold(endpoint):
    channel = MavlinkCommandChannel("127.0.0.1")
    command, params = _COMMAND_MAP[endpoint](" ")
    decoded = _decode(channel._frame_command(command, params))
    assert decoded.command == CMD_DO_SET_MODE
    assert decoded.param2 == PX4_MODE_POSCTL, "an abort must land in position hold"


def test_entering_stick_control_asks_for_offboard():
    channel = MavlinkCommandChannel("127.0.0.1")
    command, params = _COMMAND_MAP["/send/enableVirtualStick"](" ")
    decoded = _decode(channel._frame_command(command, params))
    assert decoded.command == CMD_DO_SET_MODE
    assert decoded.param2 == PX4_MODE_OFFBOARD


def test_the_two_user_commands_are_told_apart_by_their_selector():
    channel = MavlinkCommandChannel("127.0.0.1")
    cases = {
        "/send/lrf/measure": (CMD_USER_2, USER2_LRF_MEASURE),
        "/send/captureTemperature": (CMD_USER_2, USER2_CAPTURE_TEMPERATURE),
        "/send/deactivateManualOverride": (CMD_USER_1, USER1_RELEASE_MANUAL_OVERRIDE),
        "/send/gimbal/rel_pitch": (CMD_USER_1, USER1_GIMBAL_RELATIVE),
    }
    for endpoint, (expected_command, selector) in cases.items():
        command, params = _COMMAND_MAP[endpoint]("0,-15,0")
        decoded = _decode(channel._frame_command(command, params))
        assert decoded.command == expected_command, endpoint
        assert decoded.param1 == pytest.approx(selector), endpoint


def test_a_relative_gimbal_nudge_moves_only_the_axis_it_names():
    channel = MavlinkCommandChannel("127.0.0.1")
    command, params = _COMMAND_MAP["/send/gimbal/rel_pitch"]("0,-15,0")
    pitch_cmd = _decode(channel._frame_command(command, params))
    assert pitch_cmd.param2 == pytest.approx(-15.0)
    assert pitch_cmd.param3 == 0.0, "a pitch nudge must not also swing the yaw"

    command, params = _COMMAND_MAP["/send/gimbal/rel_yaw"]("0,0,30")
    yaw_cmd = _decode(channel._frame_command(command, params))
    assert yaw_cmd.param2 == 0.0
    assert yaw_cmd.param3 == pytest.approx(30.0)


def test_a_drop_is_a_gripper_release():
    channel = MavlinkCommandChannel("127.0.0.1")
    command, params = _COMMAND_MAP["/send/drop"]("")
    decoded = _decode(channel._frame_command(command, params))
    assert decoded.command == CMD_DO_GRIPPER
    assert decoded.param2 == GRIPPER_ACTION_RELEASE


def test_sticks_are_sent_as_manual_control_on_the_right_axes():
    """DJI's sticks are left=yaw/throttle, right=roll/pitch; MAVLink's axes are named by meaning.

    Getting this mapping wrong swaps yaw for roll on a flying aircraft, which is why it is pinned.
    """
    sent = {}
    channel = MavlinkCommandChannel("127.0.0.1")
    channel._socket = type("S", (), {"sendto": lambda self, b, a: sent.update(frame=b)})()
    channel.send("/send/stick", "0.1,0.2,0.3,0.4")

    decoded = _decode(sent["frame"])
    assert decoded.get_type() == "MANUAL_CONTROL"
    assert decoded.r == 100, "leftX is yaw"
    assert decoded.z == 200, "leftY is throttle"
    assert decoded.y == 300, "rightX is roll"
    assert decoded.x == 400, "rightY is pitch"


def test_manual_control_axes_are_clamped():
    sent = {}
    channel = MavlinkCommandChannel("127.0.0.1")
    channel._socket = type("S", (), {"sendto": lambda self, b, a: sent.update(frame=b)})()
    channel.send_manual_control(roll=5.0, pitch=-5.0, throttle=0.0, yaw=0.0)

    decoded = _decode(sent["frame"])
    assert decoded.y == 1000 and decoded.x == -1000


# -- the WildBridge dialect ---------------------------------------------------------------------


def test_the_hand_decoder_matches_the_dialect_definition():
    """Regenerate WILDBRIDGE_STATUS from wildbridge.xml and check our layout still matches.

    transport.py unpacks this message by hand rather than shipping a generated dialect, which is
    cheap for one message but silently wrong the moment the XML changes. This test regenerates the
    definition and compares, so the XML stays the source of truth in practice and not just in
    principle.
    """
    import pathlib
    import shutil
    import subprocess
    import sys
    import tempfile

    xml = pathlib.Path(__file__).resolve().parents[1] / "mavlink" / "wildbridge.xml"
    assert xml.exists(), xml

    pymavlink = pytest.importorskip("pymavlink")
    definitions = pathlib.Path(pymavlink.__file__).parent / "message_definitions" / "v1.0"

    with tempfile.TemporaryDirectory() as tmp:
        work = pathlib.Path(tmp)
        for source in definitions.glob("*.xml"):
            shutil.copy(source, work / source.name)
        shutil.copy(xml, work / "wildbridge.xml")

        generate = (
            "from pymavlink.generator import mavgen; import argparse; "
            "mavgen.mavgen(argparse.Namespace(language='Python', wire_protocol='2.0', "
            "output='dialect', error_limit=200, validate=False, strict_units=False), "
            "['wildbridge.xml'])"
        )
        subprocess.run([sys.executable, "-c", generate], cwd=work, check=True, capture_output=True)

        sys.path.insert(0, str(work))
        try:
            import dialect  # type: ignore[import-not-found]

            message = dialect.MAVLink_wildbridge_status_message
            assert message.id == WILDBRIDGE_STATUS_ID
            assert message.unpacker.format == WILDBRIDGE_STATUS_STRUCT
            assert message.unpacker.size == WILDBRIDGE_STATUS_SIZE
        finally:
            sys.path.remove(str(work))
            sys.modules.pop("dialect", None)


def test_wildbridge_status_decodes_into_the_http_telemetry_keys():
    payload = struct.pack(
        WILDBRIDGE_STATUS_STRUCT,
        1234,
        465180000,
        65660000,
        12.5,
        900.0,
        45,
        18,
        63,
        24,
        25,
        26,
        18,
        12,
        WB_FLAG_MANUAL_OVERRIDE | WB_FLAG_READY_TO_TAKEOFF | WB_FLAG_LRF_TARGET_VALID,
        b"NONE",
    )
    status = decode_wildbridge_status(payload)

    assert status["isManualOverrideActive"] is True
    assert status["readyToTakeoff"] is True
    assert status["homeSet"] is False
    assert status["takeoffBlockReason"] == "NONE"
    assert status["timeNeededToGoHome"] == 45
    assert status["batteryNeededToLand"] == 12
    assert (status["zoomFl"], status["opticalFl"], status["hybridFl"]) == (24, 25, 26)
    assert status["lrfTarget"] == [pytest.approx(46.518), pytest.approx(6.566), pytest.approx(12.5)]


def test_an_unlocked_rangefinder_reports_no_target_rather_than_null_island():
    payload = struct.pack(
        WILDBRIDGE_STATUS_STRUCT,
        0,
        0,
        0,
        0.0,
        0.0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        b"",
    )
    # 0,0 is a real place off West Africa; reporting it as a target would be worse than reporting
    # nothing, because a consumer cannot tell it apart from a genuine fix.
    assert decode_wildbridge_status(payload)["lrfTarget"] is None


def test_a_truncated_payload_still_decodes():
    """MAVLink 2 drops trailing zeros, so a mostly-empty status arrives short."""
    full = struct.pack(WILDBRIDGE_STATUS_STRUCT, 7, 0, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b"")
    assert decode_wildbridge_status(full.rstrip(b"\x00"))["takeoffBlockReason"] == "UNKNOWN"


# -- derived and gimbal telemetry ---------------------------------------------------------------


def test_distance_to_home_is_computed_rather_than_transmitted():
    telemetry = {
        "location": {"latitude": 46.5180, "longitude": 6.5660},
        "homeLocation": {"latitude": 46.5190, "longitude": 6.5670},
        "homeSet": True,
    }
    _derive(telemetry)
    # ~135 m north-east; checked against the great-circle distance rather than a magic number.
    assert telemetry["distanceToHome"] == pytest.approx(135.0, abs=5.0)


def test_distance_to_home_is_zero_until_home_is_known():
    telemetry = {"location": {"latitude": 46.5, "longitude": 6.5}, "homeSet": False}
    _derive(telemetry)
    assert telemetry["distanceToHome"] == 0.0


def test_gimbal_attitude_reports_the_world_frame_angles():
    telemetry = {}
    q = _euler_to_quat(0.0, math.radians(-45.0), math.radians(90.0))
    apply_mavlink_message(telemetry, FakeMessage("GIMBAL_DEVICE_ATTITUDE_STATUS", q=q))
    assert telemetry["gimbalAttitude"]["pitch"] == pytest.approx(-45.0, abs=0.01)
    assert telemetry["gimbalAttitude"]["yaw"] == pytest.approx(90.0, abs=0.01)


def test_the_joint_attitude_is_the_gimbal_relative_to_a_tilted_aircraft():
    """A gimbal holding level on a rolled aircraft is not level in the body frame.

    Subtracting euler angles happens to work here, but the code composes quaternions because the
    subtraction stops being correct exactly when the aircraft stops being close to level.
    """
    telemetry = {
        "attitude": {"roll": 1.3, "pitch": 0.1, "yaw": 0.0},
        "_gimbalQuaternion": _euler_to_quat(0.0, 0.0, 0.0),
        "gimbalAttitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    }
    _derive(telemetry)
    joint = telemetry["gimbalJointAttitude"]
    assert joint["roll"] == pytest.approx(1.3, abs=0.01)
    assert joint["pitch"] == pytest.approx(0.1, abs=0.01)


def test_the_joint_attitude_falls_back_before_any_attitude_arrives():
    telemetry = {
        "_gimbalQuaternion": _euler_to_quat(0.0, math.radians(-30.0), 0.0),
        "gimbalAttitude": {"roll": 0.0, "pitch": -30.0, "yaw": 0.0},
    }
    _derive(telemetry)
    assert telemetry["gimbalJointAttitude"]["pitch"] == pytest.approx(-30.0)


def test_unreported_focal_lengths_read_as_minus_one_not_zero():
    payload = struct.pack(
        WILDBRIDGE_STATUS_STRUCT, 0, 0, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b""
    )
    status = decode_wildbridge_status(payload)
    # 0 mm is not a lens. The HTTP stream says -1 for "not reported" and so must this.
    assert (status["zoomFl"], status["opticalFl"], status["hybridFl"]) == (-1, -1, -1)


def test_the_rangefinder_range_arrives_as_a_standard_distance_sensor():
    telemetry = {}
    apply_mavlink_message(telemetry, FakeMessage("DISTANCE_SENSOR", current_distance=4250))
    assert telemetry["lrfDistance"] == pytest.approx(42.5)


def _euler_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


# -- completion reporting -----------------------------------------------------------------------


def test_a_goto_returns_at_once_and_raises_its_latch_when_it_arrives():
    """Accepted is not arrived, but the caller must not be made to wait for arrival either.

    The HTTP contract is that a goto returns a seq immediately and the caller polls the reach
    latch. Blocking here until the aircraft arrived would stall a ROS callback for the whole
    flight, so the completion is picked up in the background and raises the latch there.
    """
    channel = MavlinkCommandChannel("127.0.0.1")
    latched = {}
    channel._on_latch = latched.update
    channel._socket = _FakeSocket(
        [
            _ack_frame(CMD_DO_REPOSITION, MAV_RESULT_IN_PROGRESS),
            _ack_frame(CMD_DO_REPOSITION, 0),
        ]
    )

    started = time.time()
    reply = channel.send("/send/gotoWaypointHoldHeading", "46.518,6.566,30,90,5", timeout=2.0)
    elapsed = time.time() - started

    assert reply.startswith("ACCEPTED")
    assert elapsed < 1.0, "the caller must not wait for the flight"

    seq = int(reply.split("seq=")[1].split()[0])
    deadline = time.time() + 3.0
    while time.time() < deadline and "waypointReached" not in latched:
        time.sleep(0.02)

    assert latched.get("waypointReached") is True
    assert latched.get("waypointSeq") == seq, "the arrival must be reported under its own seq"


def test_a_command_that_never_completes_leaves_its_latch_down():
    """An accepted command that never arrives must not look like an arrival."""
    channel = MavlinkCommandChannel("127.0.0.1")
    latched = {}
    channel._on_latch = latched.update
    channel._socket = _FakeSocket([_ack_frame(CMD_DO_REPOSITION, MAV_RESULT_IN_PROGRESS)])

    reply = channel.send("/send/gotoWaypointHoldHeading", "46.518,6.566,30,90,5", timeout=1.0)

    assert reply.startswith("ACCEPTED"), "it was accepted; it just has not finished"
    time.sleep(0.3)
    assert "waypointReached" not in latched


def test_a_refused_command_is_reported_as_refused():
    channel = MavlinkCommandChannel("127.0.0.1")
    channel._socket = _FakeSocket([_ack_frame(CMD_DO_REPOSITION, 3)])  # MAV_RESULT_DENIED
    reply = channel.send("/send/gotoWaypointHoldHeading", "46.518,6.566,30,90,5", timeout=1.0)
    assert reply.startswith("REJECTED")


def _ack_frame(command: int, result: int) -> bytes:
    from pymavlink.dialects.v20 import common as mavlink_common

    class Sink:
        buf = b""

        def write(self, data):
            self.buf += data

    sink = Sink()
    mavlink_common.MAVLink(sink, srcSystem=1, srcComponent=1).command_ack_send(command, result)
    return sink.buf


class _FakeSocket:
    """Hands back queued frames, then behaves like a socket with nothing to read."""

    def __init__(self, frames):
        self._frames = frames

    def sendto(self, data, address):
        return len(data)

    def settimeout(self, timeout):
        return None

    def recvfrom(self, size):
        if not self._frames:
            # A real socket blocks until its timeout; raising instantly would spin the reader.
            time.sleep(0.02)
            raise TimeoutError
        return self._frames.pop(0), ("127.0.0.1", 14550)
