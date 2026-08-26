"""Which wire the ground station talks to WildBridge over.

WildBridge has spoken HTTP since the beginning: commands are POSTs to port 8080, telemetry is a
JSON stream on 8081. The MAVLink endpoint is newer and covers a growing subset of the same ground.
Rather than fork the client, this module lets the existing :class:`DJIInterface` swap the wire
underneath itself, because the whole surface funnels through exactly two chokepoints — the
telemetry dictionary and ``requestSend`` — and everything above them (every getter, every ROS
publisher, every script) reads that dictionary and calls those methods without caring how they
were filled.

Select the wire with the ``WB_TRANSPORT`` environment variable:

``http``
    Today's behaviour, and the default. Nothing touches MAVLink.
``mavlink``
    Telemetry from the MAVLink stream, commands as MAVLink. Anything MAVLink does not yet
    implement is **refused, not quietly retried over HTTP** — the point of this mode is to make
    the remaining gaps visible, and a silent fallback would hide exactly what we are trying to
    measure.
``both``
    Telemetry from MAVLink, commands over MAVLink where it has an equivalent and HTTP where it
    does not. This is the migration mode: it exercises the new wire while keeping the full
    command surface working.
"""

from __future__ import annotations

import math
import os
import socket
import threading
import time
from collections.abc import Callable
from contextlib import suppress
from enum import Enum
from typing import Any

from wildbridge_groundstation.mavlink_helpers import gps_fix_type  # noqa: F401

ENV_VAR = "WB_TRANSPORT"

#: MAV_COMP_ID_AUTOPILOT1. Vehicle state is only believed from this component.
COMP_ID_AUTOPILOT1 = 1
DEFAULT_MAVLINK_PORT = 14550

#: Returned by the MAVLink command channel when an endpoint has no MAVLink equivalent. Callers
#: parse command replies as strings, so this is a string rather than an exception: it flows
#: through the same path a rejection from the aircraft would.
UNSUPPORTED_PREFIX = "REJECTED: no MAVLink equivalent for"


def mavlink_port_from_env(environ: dict[str, str] | None = None) -> int:
    """UDP port this ground station listens on, from ``WB_MAVLINK_PORT``.

    Configurable because a fleet needs one port per aircraft: only one socket receives a given
    UDP port's packets, so two aircraft sharing a port would leave one of them unread.
    """
    raw = (environ if environ is not None else os.environ).get("WB_MAVLINK_PORT", "").strip()
    if not raw:
        return DEFAULT_MAVLINK_PORT
    try:
        return int(raw)
    except ValueError:
        raise ValueError(f"WB_MAVLINK_PORT={raw!r} is not a port number") from None


class Transport(Enum):
    """Which wire to use."""

    HTTP = "http"
    MAVLINK = "mavlink"
    BOTH = "both"

    @classmethod
    def from_env(cls, environ: dict[str, str] | None = None) -> Transport:
        """Read ``WB_TRANSPORT``, defaulting to HTTP.

        An unrecognised value is a configuration mistake worth failing on rather than silently
        treating as the default: a typo'd ``WB_TRANSPORT=mavlnk`` that quietly ran over HTTP would
        produce a test result that looks like a passing MAVLink run.
        """
        raw = (environ if environ is not None else os.environ).get(ENV_VAR, "").strip().lower()
        if not raw:
            return cls.HTTP
        try:
            return cls(raw)
        except ValueError:
            valid = ", ".join(t.value for t in cls)
            raise ValueError(f"{ENV_VAR}={raw!r} is not one of: {valid}") from None

    @property
    def uses_mavlink(self) -> bool:
        return self in (Transport.MAVLINK, Transport.BOTH)

    @property
    def allows_http_fallback(self) -> bool:
        """Whether a command with no MAVLink equivalent may go out over HTTP instead."""
        return self is not Transport.MAVLINK


# --------------------------------------------------------------------------------------------
# Telemetry
# --------------------------------------------------------------------------------------------


def _quat_to_euler_deg(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Convert a MAVLink attitude quaternion to DJI's degrees. Unused today, kept for gimbals."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


#: PX4 packed custom_mode values (``(main << 16) | (sub << 24)``) back to the mode names the HTTP
#: telemetry stream reports. The aircraft claims MAV_AUTOPILOT_PX4 so that QGroundControl enables
#: its guided-mode buttons, which means custom_mode arrives in PX4's packing rather than DJI's
#: names; this table is the inverse of MavlinkFlightMode on the aircraft side.
PX4_MODE_NAMES = {
    1 << 16: "MANUAL",
    2 << 16: "ATTI",
    3 << 16: "GPS_NORMAL",
    6 << 16: "VIRTUAL_STICK",
    (3 << 16) | (1 << 24): "POI",
    (4 << 16) | (4 << 24): "WAYPOINT",
    (4 << 16) | (2 << 24): "AUTO_TAKE_OFF",
    (4 << 16) | (6 << 24): "AUTO_LANDING",
    (4 << 16) | (5 << 24): "GO_HOME",
    (4 << 16) | (8 << 24): "SMART_FLIGHT",
}


def flight_mode_name(custom_mode: int) -> str:
    """Name for a PX4-packed custom_mode, or UNKNOWN when it is not one we advertise."""
    return PX4_MODE_NAMES.get(int(custom_mode), "UNKNOWN")


def _dji_heading(hdg_centidegrees: int) -> float:
    """MAVLink heading (0..360) in DJI's convention (-180..180).

    Both describe the same bearing, but a consumer comparing the two -- or switching transports
    mid-flight -- would see a 360-degree jump at north if the conventions were left to differ.
    65535 is MAVLink's "unknown".
    """
    if hdg_centidegrees == 65535:
        return 0.0
    degrees = hdg_centidegrees / 100.0
    return degrees - 360.0 if degrees > 180.0 else degrees


def _from_heartbeat(telemetry: dict[str, Any], msg: Any) -> bool:
    # A WildBridge aircraft emits more than one heartbeat: the autopilot's, and the camera
    # component's at MAV_COMP_ID_CAMERA. The camera's custom_mode is a meaningless zero, so taking
    # whichever arrived last silently reports UNKNOWN for a drone in position hold. QGroundControl
    # filters the same way (MultiVehicleManager drops any heartbeat whose component is not
    # MAV_COMP_ID_AUTOPILOT1) and for the same reason.
    if msg.get_header().srcComponent != COMP_ID_AUTOPILOT1:
        return False
    telemetry["flightMode"] = flight_mode_name(msg.custom_mode)
    return True


def _from_global_position(telemetry: dict[str, Any], msg: Any) -> bool:
    telemetry["location"] = {
        "latitude": msg.lat / 1e7,
        "longitude": msg.lon / 1e7,
        # DJI reports altitude above takeoff, which is MAVLink's relative_alt.
        "altitude": msg.relative_alt / 1000.0,
    }
    # The HTTP stream carries altitude both nested and at the top level; consumers read both.
    telemetry["altitude"] = msg.relative_alt / 1000.0
    telemetry["heading"] = _dji_heading(msg.hdg)
    telemetry["speed"] = {
        "x": msg.vx / 100.0,
        "y": msg.vy / 100.0,
        # DJI's z is positive-down, and so is MAVLink's vz.
        "z": msg.vz / 100.0,
    }
    return True


def _from_attitude(telemetry: dict[str, Any], msg: Any) -> bool:
    telemetry["attitude"] = {
        "roll": math.degrees(msg.roll),
        "pitch": math.degrees(msg.pitch),
        "yaw": math.degrees(msg.yaw),
    }
    return True


def _from_gps_raw(telemetry: dict[str, Any], msg: Any) -> bool:
    telemetry["satelliteCount"] = msg.satellites_visible
    return True


def _from_sys_status(telemetry: dict[str, Any], msg: Any) -> bool:
    telemetry["batteryLevel"] = msg.battery_remaining
    return True


def _from_battery_status(telemetry: dict[str, Any], msg: Any) -> bool:
    telemetry["batteryLevel"] = msg.battery_remaining
    if getattr(msg, "time_remaining", 0):
        telemetry["remainingFlightTime"] = msg.time_remaining
    return True


def _from_home_position(telemetry: dict[str, Any], msg: Any) -> bool:
    telemetry["homeLocation"] = {
        "latitude": msg.latitude / 1e7,
        "longitude": msg.longitude / 1e7,
        "altitude": msg.altitude / 1000.0,
    }
    # HOME_POSITION is only emitted once the aircraft actually has a home point.
    telemetry["homeSet"] = True
    return True


def _from_vfr_hud(telemetry: dict[str, Any], msg: Any) -> bool:
    telemetry["speed"] = dict(telemetry.get("speed", {}), z=-msg.climb)
    telemetry["altitude"] = msg.alt
    return True


def _from_capture_status(telemetry: dict[str, Any], msg: Any) -> bool:
    # video_status: 0 idle, 1 capturing.
    telemetry["isRecording"] = msg.video_status == 1
    telemetry.setdefault("zoomRatio", 1.0)
    return True


def _from_mission_current(telemetry: dict[str, Any], msg: Any) -> bool:
    telemetry["missionCurrent"] = msg.seq
    telemetry["missionState"] = msg.mission_state
    return True


def _from_mission_item_reached(telemetry: dict[str, Any], msg: Any) -> bool:
    # The mission protocol's arrival report is the MAVLink-side equivalent of the reach latch the
    # HTTP surface exposes as waypointReached/waypointSeq.
    telemetry["waypointReached"] = True
    telemetry["waypointSeq"] = msg.seq
    return True


#: One handler per message we translate. A table rather than a chain of branches, matching how
#: mavlink_listen.py dispatches, and so that adding a message is a single entry.
_TELEMETRY_HANDLERS: dict[str, Callable[[dict[str, Any], Any], bool]] = {
    "HEARTBEAT": _from_heartbeat,
    "GLOBAL_POSITION_INT": _from_global_position,
    "ATTITUDE": _from_attitude,
    "GPS_RAW_INT": _from_gps_raw,
    "SYS_STATUS": _from_sys_status,
    "BATTERY_STATUS": _from_battery_status,
    "HOME_POSITION": _from_home_position,
    "VFR_HUD": _from_vfr_hud,
    "CAMERA_CAPTURE_STATUS": _from_capture_status,
    "MISSION_CURRENT": _from_mission_current,
    "MISSION_ITEM_REACHED": _from_mission_item_reached,
}


def apply_mavlink_message(telemetry: dict[str, Any], msg: Any) -> bool:
    """Fold one decoded MAVLink message into the telemetry dictionary.

    The keys written here are deliberately the ones WildBridge's HTTP telemetry stream already
    produces, so a consumer cannot tell which wire filled them. Where MAVLink carries a field the
    HTTP stream does not, it is dropped rather than invented under a new name -- a consumer that
    learns to read a MAVLink-only key stops working the moment the transport changes back.

    Returns True when the message changed anything, which is what drives the sequence counter.
    """
    handler = _TELEMETRY_HANDLERS.get(msg.get_type())
    return handler(telemetry, msg) if handler is not None else False


class MavlinkTelemetrySource:
    """Listens for the aircraft's MAVLink stream and keeps a telemetry dictionary current.

    Runs its own receive thread, like the HTTP telemetry reader it stands in for, and writes
    through the callback it is given so the owning client keeps a single lock over its state.
    """

    def __init__(
        self,
        port: int = DEFAULT_MAVLINK_PORT,
        bind_host: str = "",
        on_update: Callable[[dict[str, Any]], None] | None = None,
        peer_host: str = "",
    ):
        self.port = port
        self.bind_host = bind_host
        #: Only accept packets from this aircraft. A fleet is the reason: several aircraft
        #: streaming to one port would be folded into a single telemetry dictionary, and the
        #: result would look like one drone teleporting between positions rather than like an
        #: error. Empty means accept anything, which is right for a single aircraft.
        self.peer_host = peer_host
        self._on_update = on_update
        self._telemetry: dict[str, Any] = {}
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        #: Address the last packet came from, so commands can be sent back to the aircraft even
        #: when its address was never configured.
        self.peer: tuple[str, int] | None = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._socket is not None:
            with suppress(OSError):
                self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def _receive_loop(self) -> None:
        from pymavlink.dialects.v20 import common as mavlink_common

        parser = mavlink_common.MAVLink(None)
        parser.robust_parsing = True

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self.bind_host, self.port))
        self._socket.settimeout(1.0)

        while self._running:
            try:
                data, addr = self._socket.recvfrom(2048)
            except TimeoutError:
                continue
            except OSError:
                break
            if self.peer_host and addr[0] != self.peer_host:
                continue
            self.peer = addr
            for msg in parser.parse_buffer(data) or []:
                if apply_mavlink_message(self._telemetry, msg) and self._on_update is not None:
                    self._on_update(dict(self._telemetry))


# --------------------------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------------------------

# MAV_CMD numbers, matching the set MavlinkTelemetryEndpoint actually handles. Anything absent
# here is absent on the aircraft too, so listing more would only move the refusal one hop later.
CMD_NAV_RETURN_TO_LAUNCH = 20
CMD_NAV_LAND = 21
CMD_NAV_TAKEOFF = 22
CMD_CONDITION_YAW = 115
CMD_DO_REPOSITION = 192
CMD_MISSION_START = 300
CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
CMD_IMAGE_START_CAPTURE = 2000
CMD_VIDEO_START_CAPTURE = 2500
CMD_VIDEO_STOP_CAPTURE = 2501
CMD_SET_CAMERA_ZOOM = 531

CMD_DO_SET_MODE = 176
CMD_CONDITION_CHANGE_ALT = 113
CMD_DO_GRIPPER = 211
CMD_USER_1 = 31010
CMD_USER_2 = 31011
CMD_MISSION_START = 300

#: PX4 packed custom_mode values the aircraft advertises, used to request a mode.
PX4_MODE_POSCTL = 3 << 16
PX4_MODE_OFFBOARD = 6 << 16

#: USER_1 / USER_2 selectors, carried in param1. Kept in step with Mav in MavlinkProtocol.kt.
USER1_GIMBAL_RELATIVE = 1.0
USER1_RELEASE_MANUAL_OVERRIDE = 2.0
USER2_LRF_MEASURE = 1.0
USER2_CAPTURE_TEMPERATURE = 2.0

GRIPPER_ACTION_RELEASE = 0

MAV_RESULT_ACCEPTED = 0

#: MAV_FRAME_GLOBAL_RELATIVE_ALT_INT: altitudes are metres above the home point, which is what
#: DJI reports and what every WildBridge altitude already means.
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

#: Endpoints whose MAVLink form is a plain COMMAND_LONG. The builder turns the HTTP payload
#: string into the seven command parameters, so the mapping stays in one readable place.
_COMMAND_MAP: dict[str, Callable[[str], tuple[int, list[float]]]] = {}


def _numbers(payload: str) -> list[float]:
    """The comma-separated numbers in an HTTP command payload."""
    return [float(part) for part in str(payload).split(",") if part.strip()]


def _register(endpoint: str) -> Callable[..., Any]:
    def wrap(
        fn: Callable[[str], tuple[int, list[float]]],
    ) -> Callable[[str], tuple[int, list[float]]]:
        _COMMAND_MAP[endpoint] = fn
        return fn

    return wrap


@_register("/send/takeoff")
def _takeoff(payload: str) -> tuple[int, list[float]]:
    # param7 is the requested altitude; 0 tells the aircraft to use its own default.
    altitude = _numbers(payload)[0] if _numbers(payload) else 0.0
    return CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, altitude]


@_register("/send/land")
def _land(_payload: str) -> tuple[int, list[float]]:
    return CMD_NAV_LAND, [0] * 7


@_register("/send/RTH")
def _rth(_payload: str) -> tuple[int, list[float]]:
    return CMD_NAV_RETURN_TO_LAUNCH, [0] * 7


@_register("/send/gotoYaw")
def _goto_yaw(payload: str) -> tuple[int, list[float]]:
    return CMD_CONDITION_YAW, [_numbers(payload)[0], 0, 0, 0, 0, 0, 0]


@_register("/send/camera/zoom")
def _zoom(payload: str) -> tuple[int, list[float]]:
    # param1 selects the zoom type (2 = absolute ratio), param2 carries the value.
    return CMD_SET_CAMERA_ZOOM, [2, _numbers(payload)[0], 0, 0, 0, 0, 0]


@_register("/send/camera/startRecording")
def _start_recording(_payload: str) -> tuple[int, list[float]]:
    return CMD_VIDEO_START_CAPTURE, [0] * 7


@_register("/send/camera/stopRecording")
def _stop_recording(_payload: str) -> tuple[int, list[float]]:
    return CMD_VIDEO_STOP_CAPTURE, [0] * 7


@_register("/send/capture")
def _capture(_payload: str) -> tuple[int, list[float]]:
    # One frame, no interval, sequence number 1.
    return CMD_IMAGE_START_CAPTURE, [0, 0, 1, 1, 0, 0, 0]


@_register("/send/gimbal/pitch")
def _gimbal_pitch(payload: str) -> tuple[int, list[float]]:
    values = _numbers(payload)
    pitch = values[1] if len(values) > 1 else 0.0
    return CMD_DO_GIMBAL_MANAGER_PITCHYAW, [pitch, float("nan"), 0, 0, 0, 0, 0]


@_register("/send/gimbal/yaw")
def _gimbal_yaw(payload: str) -> tuple[int, list[float]]:
    values = _numbers(payload)
    yaw = values[2] if len(values) > 2 else 0.0
    return CMD_DO_GIMBAL_MANAGER_PITCHYAW, [float("nan"), yaw, 0, 0, 0, 0, 0]


#: Commands whose position must survive the trip. MAVLink defines COMMAND_INT precisely for
#: these: it carries x/y as int32 degE7 instead of float32, which at mid latitudes is the
#: difference between roughly 0.6 m of rounding error and none.
POSITION_COMMANDS = frozenset({CMD_DO_REPOSITION})


def _waypoint(payload: str, nose_forward: bool) -> tuple[int, list[float]]:
    """A single goto, as DO_REPOSITION.

    param4 carries the heading with the same meaning it has in a mission item: NaN asks the
    aircraft for nose-along-path, a value asks for that heading to be held. The aircraft honours
    the distinction in both places, so a goto does not change character depending on whether it
    arrived as a reposition or as a one-item plan.
    """
    latitude, longitude, altitude, yaw, speed = (_numbers(payload) + [0] * 5)[:5]
    heading = float("nan") if nose_forward else yaw
    return CMD_DO_REPOSITION, [speed, 0, 0, heading, latitude, longitude, altitude]


@_register("/send/abortMission")
@_register("/send/abortAll")
@_register("/send/abort/DJIMission")
def _abort(_payload: str) -> tuple[int, list[float]]:
    """Every abort is a mode change to position hold.

    The HTTP surface has three with different scopes; MAVLink expresses an abort as a mode, which
    has one meaning. The aircraft does the union of the three, so widening is the only direction
    this can be wrong in.
    """
    # param1 is base_mode with CUSTOM_MODE_ENABLED, param2 the PX4 custom mode.
    return CMD_DO_SET_MODE, [1, PX4_MODE_POSCTL, 0, 0, 0, 0, 0]


@_register("/send/enableVirtualStick")
def _enable_virtual_stick(_payload: str) -> tuple[int, list[float]]:
    # Offboard is the standard name for the mode that accepts stick input.
    return CMD_DO_SET_MODE, [1, PX4_MODE_OFFBOARD, 0, 0, 0, 0, 0]


@_register("/send/gotoAltitude")
def _goto_altitude(payload: str) -> tuple[int, list[float]]:
    # param1 is the climb rate, which DJI chooses; param7 the target altitude.
    return CMD_CONDITION_CHANGE_ALT, [0, 0, 0, 0, 0, 0, _numbers(payload)[0]]


@_register("/send/deactivateManualOverride")
def _release_override(_payload: str) -> tuple[int, list[float]]:
    return CMD_USER_1, [USER1_RELEASE_MANUAL_OVERRIDE, 0, 0, 0, 0, 0, 0]


@_register("/send/gimbal/rel_pitch")
def _gimbal_rel_pitch(payload: str) -> tuple[int, list[float]]:
    values = _numbers(payload)
    pitch = values[1] if len(values) > 1 else 0.0
    return CMD_USER_1, [USER1_GIMBAL_RELATIVE, pitch, 0, 0, 0, 0, 0]


@_register("/send/gimbal/rel_yaw")
def _gimbal_rel_yaw(payload: str) -> tuple[int, list[float]]:
    values = _numbers(payload)
    yaw = values[2] if len(values) > 2 else 0.0
    return CMD_USER_1, [USER1_GIMBAL_RELATIVE, 0, yaw, 0, 0, 0, 0]


@_register("/send/lrf/measure")
def _lrf_measure(_payload: str) -> tuple[int, list[float]]:
    return CMD_USER_2, [USER2_LRF_MEASURE, 0, 0, 0, 0, 0, 0]


@_register("/send/captureTemperature")
def _capture_temperature(_payload: str) -> tuple[int, list[float]]:
    return CMD_USER_2, [USER2_CAPTURE_TEMPERATURE, 0, 0, 0, 0, 0, 0]


@_register("/send/drop")
def _drop(_payload: str) -> tuple[int, list[float]]:
    # param1 is the gripper instance, param2 the action. WildBridge has one drop port.
    return CMD_DO_GRIPPER, [1, GRIPPER_ACTION_RELEASE, 0, 0, 0, 0, 0]


_COMMAND_MAP["/send/gotoWaypointNoseForward"] = lambda p: _waypoint(p, nose_forward=True)
_COMMAND_MAP["/send/gotoWaypointHoldHeading"] = lambda p: _waypoint(p, nose_forward=False)


def _send_stick(channel: MavlinkCommandChannel, payload: str, _timeout: float) -> str:
    """Stick input, as MANUAL_CONTROL.

    A message rather than a command, because that is what MAVLink defines sticks to be, and it is
    why nothing is waited for: MANUAL_CONTROL has no ack, and a stream of stick samples that each
    blocked on a reply would not be a stream.
    """
    left_x, left_y, right_x, right_y = (_numbers(payload) + [0.0] * 4)[:4]
    channel.send_manual_control(roll=right_x, pitch=right_y, throttle=left_y, yaw=left_x)
    return "ACCEPTED via MAVLink MANUAL_CONTROL"


def _send_trajectory(channel: MavlinkCommandChannel, payload: str, timeout: float) -> str:
    """A DJI native trajectory, as a mission upload followed by MISSION_START.

    This is the one endpoint whose MAVLink form is a conversation rather than a message. It is
    also the one that stops being special once the aircraft is configured with
    ``WB_MISSION_EXEC=dji_native``: the plan uploaded here is an ordinary MAVLink plan, and the
    aircraft decides it should be flown by DJI's wayline engine.
    """
    parts = str(payload).split(";")
    if len(parts) < 3:
        return "REJECTED: need a speed and at least two waypoints"
    try:
        speed = float(parts[0])
        waypoints = [tuple(float(v) for v in leg.split(",")) for leg in parts[1:]]
    except ValueError:
        return "REJECTED: malformed trajectory"

    if not channel.upload_plan(waypoints, speed, timeout):
        return "REJECTED: mission upload was not accepted"
    result, _ = channel._send_command(CMD_MISSION_START, [0, 0, 0, 0, 0, 0, 0], timeout)
    if result != MAV_RESULT_ACCEPTED:
        return f"REJECTED: MISSION_START returned MAV_RESULT {result}"
    return f"ACCEPTED via MAVLink mission, {len(waypoints)} waypoints"


def _send_rth_altitude(channel: MavlinkCommandChannel, payload: str, timeout: float) -> str:
    values = _numbers(payload)
    if not values:
        return "REJECTED: invalid altitude value"
    return channel.set_parameter(PARAM_RTH_ALTITUDE, values[0], timeout)


def _send_setting(channel: MavlinkCommandChannel, payload: str, timeout: float) -> str:
    """A named setting write, as PARAM_SET.

    The HTTP payload is ``name,value``. Only settings the aircraft publishes as writable are
    accepted; anything else comes back refused rather than silently ignored.
    """
    name, _, raw = str(payload).partition(",")
    try:
        value = float(raw)
    except ValueError:
        return f"REJECTED: {name} needs a numeric value over MAVLink"
    return channel.set_parameter(SETTING_PARAMETERS.get(name.strip(), name.strip()), value, timeout)


#: Endpoints whose MAVLink form is not a single COMMAND_LONG.
_SPECIAL_SENDERS: dict[str, Any] = {
    "/send/stick": _send_stick,
    "/send/navigateTrajectoryDJINative": _send_trajectory,
    "/send/setRTHAltitude": _send_rth_altitude,
    "/send/setSetting": _send_setting,
}

#: The one parameter the aircraft accepts a write to, and the setting names that map onto it.
PARAM_RTH_ALTITUDE = "WB_RTH_ALT"

#: What a WildBridge parameter reads when the aircraft has never reported it.
PARAM_UNKNOWN = -1.0
SETTING_PARAMETERS = {"rthAltitude": PARAM_RTH_ALTITUDE}


class _FrameSink:
    """Collects the bytes pymavlink writes, so a message can be framed without a live socket."""

    def __init__(self) -> None:
        self.buf = b""

    def write(self, data: bytes) -> None:
        self.buf += data


class MavlinkCommandChannel:
    """Sends commands to the aircraft as MAVLink and waits for the COMMAND_ACK.

    The reply is rendered as the same kind of string the HTTP surface returns, because that is
    what every caller already parses — including ``_parseSeq``, which reads a ``seq=<n>`` out of a
    waypoint acceptance. MAVLink has no such per-command id, so the channel assigns one; it is
    unique per client and monotonic, which is all the stale-latch guard needs.
    """

    def __init__(self, host: str, port: int = DEFAULT_MAVLINK_PORT, target_system: int = 1):
        self.host = host
        self.port = port
        self.target_system = target_system
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()
        self._seq = 0
        # Built lazily so importing this module does not require pymavlink until a command is
        # actually sent -- the telemetry-only paths do not need it at import time.
        self._mav: Any | None = None
        self._parser: Any | None = None
        self._sink = _FrameSink()

    def supports(self, endpoint: str) -> bool:
        return endpoint in _COMMAND_MAP

    def send(self, endpoint: str, payload: str, timeout: float = 3.0) -> str:
        """Send one command and render the reply the way the HTTP surface would.

        Callers parse command replies as strings — ``_parseSeq`` reads a ``seq=<n>`` out of a
        waypoint acceptance, the LRF and temperature callers parse JSON — so the shapes here match
        what the same endpoint returns over HTTP rather than inventing a MAVLink-flavoured one.
        """
        handler = _SPECIAL_SENDERS.get(endpoint)
        if handler is not None:
            return handler(self, payload, timeout)

        builder = _COMMAND_MAP.get(endpoint)
        if builder is None:
            return f"{UNSUPPORTED_PREFIX} {endpoint}"

        command, params = builder(payload)
        result, value = self._send_command(command, params, timeout)
        return self._render(endpoint, command, result, value)

    def _render(self, endpoint: str, command: int, result: int | None, value: int) -> str:
        """Turn a COMMAND_ACK into the text this endpoint returns over HTTP."""
        if endpoint == "/send/lrf/measure":
            if result != MAV_RESULT_ACCEPTED:
                return '{"distance": null, "target": null, "state": null}'
            # The geo-referenced target arrives on the telemetry stream as lrfTarget, exactly as
            # it does over HTTP; the ack carries only the distance.
            return f'{{"distance": {value / 100.0}, "target": null, "state": "NORMAL"}}'

        if endpoint == "/send/captureTemperature":
            if result != MAV_RESULT_ACCEPTED:
                return '{"thermalMaxTemp": null}'
            return f'{{"thermalMaxTemp": {value / 100.0}}}'

        if result is None:
            # No ack arrived. Reported rather than assumed: a caller that waits on a reach latch
            # would otherwise sit there believing a command it never confirmed.
            return f"REJECTED: no COMMAND_ACK for {endpoint}"
        if result != MAV_RESULT_ACCEPTED:
            return f"REJECTED: {endpoint} returned MAV_RESULT {result}"

        with self._lock:
            self._seq += 1
            seq = self._seq
        return f"ACCEPTED seq={seq} via MAVLink cmd={command}"

    def _send_command(
        self, command: int, params: list[float], timeout: float
    ) -> tuple[int | None, int]:
        """Send a command and wait for its ack. Returns (MAV_RESULT or None, result_param2)."""
        frame = self._frame_command(command, params)
        with self._lock:
            self._socket.sendto(frame, (self.host, self.port))
        ack = self._await(lambda m: m.get_type() == "COMMAND_ACK" and m.command == command, timeout)
        if ack is None:
            return None, 0
        return ack.result, getattr(ack, "result_param2", 0) or 0

    def _await(self, matches, timeout: float):
        """Wait for a message this channel cares about, ignoring the telemetry flowing past."""
        parser = self._ensure_parser()
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._socket.settimeout(max(0.05, deadline - time.time()))
            try:
                data, _ = self._socket.recvfrom(2048)
            except (TimeoutError, OSError):
                return None
            for msg in parser.parse_buffer(data) or []:
                if matches(msg):
                    return msg
        return None

    def _ensure_parser(self):
        from pymavlink.dialects.v20 import common as mavlink_common

        if self._parser is None:
            self._parser = mavlink_common.MAVLink(None)
            self._parser.robust_parsing = True
        return self._parser

    def send_manual_control(self, roll: float, pitch: float, throttle: float, yaw: float) -> None:
        """Send one stick sample. Axes are -1..1 and are scaled to MAVLink's -1000..1000."""
        mav = self._ensure_mav()
        self._sink.buf = b""

        def axis(value: float) -> int:
            return round(max(-1.0, min(1.0, value)) * 1000)

        mav.manual_control_send(
            self.target_system, axis(pitch), axis(roll), axis(throttle), axis(yaw), 0
        )
        with self._lock:
            self._socket.sendto(self._sink.buf, (self.host, self.port))

    def set_parameter(self, name: str, value: float, timeout: float = 3.0) -> str:
        """Write one parameter and report what the aircraft says it now holds.

        MAVLink answers a write with a PARAM_VALUE carrying the current value, which is how a
        refused or clamped write is detected. Comparing rather than assuming is the point: an
        aircraft that refuses the write still replies, with the old value.
        """
        mav = self._ensure_mav()
        self._sink.buf = b""
        mav.param_set_send(
            self.target_system, COMP_ID_AUTOPILOT1, name.encode()[:16], float(value), 9
        )
        with self._lock:
            self._socket.sendto(self._sink.buf, (self.host, self.port))

        reply = self._await(
            lambda m: m.get_type() == "PARAM_VALUE" and m.param_id.rstrip("\x00") == name,
            timeout,
        )
        if reply is None:
            return f"REJECTED: no PARAM_VALUE for {name}"
        if reply.param_value == PARAM_UNKNOWN:
            # Some DJI keys are never reported spontaneously, so the aircraft genuinely does not
            # know what it holds and publishes -1. The write was accepted, but saying it was
            # confirmed would be a claim nothing supports.
            return f"ACCEPTED (unverified: the aircraft does not report {name})"
        if abs(reply.param_value - value) > 1e-3:
            return f"REJECTED: {name} is {reply.param_value}, not {value}"
        return f"{name} set to {reply.param_value}"

    def upload_plan(
        self, waypoints: list[tuple[float, ...]], speed: float, timeout: float = 6.0
    ) -> bool:
        """Upload a plan: a speed item, then one waypoint per leg. True when acked ACCEPTED."""
        mav = self._ensure_mav()
        count = len(waypoints) + 1

        def transmit(fn, *args) -> None:
            self._sink.buf = b""
            fn(*args)
            with self._lock:
                self._socket.sendto(self._sink.buf, (self.host, self.port))

        transmit(mav.mission_count_send, self.target_system, COMP_ID_AUTOPILOT1, count, 0)
        for _ in range(count + 2):
            reply = self._await(
                lambda m: m.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"),
                timeout,
            )
            if reply is None:
                return False
            if reply.get_type() == "MISSION_ACK":
                return reply.type == 0
            if reply.seq == 0:
                # DO_CHANGE_SPEED: param1 selects ground speed, param2 carries it.
                transmit(
                    mav.mission_item_int_send,
                    self.target_system,
                    COMP_ID_AUTOPILOT1,
                    0,
                    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    178,
                    0,
                    1,
                    1.0,
                    speed,
                    -1.0,
                    0.0,
                    0,
                    0,
                    0.0,
                    0,
                )
            else:
                latitude, longitude, altitude = (list(waypoints[reply.seq - 1]) + [0.0] * 3)[:3]
                transmit(
                    mav.mission_item_int_send,
                    self.target_system,
                    COMP_ID_AUTOPILOT1,
                    reply.seq,
                    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    16,
                    0,
                    1,
                    0.0,
                    0.0,
                    0.0,
                    float("nan"),
                    round(latitude * 1e7),
                    round(longitude * 1e7),
                    altitude,
                    0,
                )
        return False

    def _frame_command(self, command: int, params: list[float]) -> bytes:
        """Encode one command, as COMMAND_INT when it carries a position and COMMAND_LONG else."""
        mav = self._ensure_mav()
        self._sink.buf = b""
        values = [float(p) for p in (list(params) + [0.0] * 7)[:7]]

        if command in POSITION_COMMANDS:
            mav.command_int_send(
                self.target_system,
                COMP_ID_AUTOPILOT1,
                MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                command,
                0,  # current
                0,  # autocontinue
                *values[:4],
                round(values[4] * 1e7),
                round(values[5] * 1e7),
                values[6],
            )
        else:
            mav.command_long_send(self.target_system, COMP_ID_AUTOPILOT1, command, 0, *values)
        return self._sink.buf

    def _ensure_mav(self) -> Any:
        """The pymavlink encoder, created on first use."""
        from pymavlink.dialects.v20 import common as mavlink_common

        if self._mav is None:
            self._mav = mavlink_common.MAVLink(self._sink, srcSystem=255, srcComponent=190)
        return self._mav
