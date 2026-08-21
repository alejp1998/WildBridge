"""Canonical WildBridge DJI HTTP/TCP client."""

from __future__ import annotations

import json
import socket
import threading
import time
from collections.abc import Callable
from contextlib import suppress
from datetime import datetime
from typing import Any

import requests

from wildbridge_groundstation.dji_helpers import (
    build_command_url,
    parse_discovery_response,
    parse_telemetry_chunk,
)

DISCOVERY_PORT = 30000
DISCOVERY_MSG = b"DISCOVER_WILDBRIDGE"

EP_STICK = "/send/stick"
EP_ZOOM = "/send/camera/zoom"
EP_GIMBAL_SET_PITCH = "/send/gimbal/pitch"
EP_GIMBAL_SET_YAW = "/send/gimbal/yaw"
EP_TAKEOFF = "/send/takeoff"
EP_LAND = "/send/land"
EP_RTH = "/send/RTH"
EP_ENABLE_VIRTUAL_STICK = "/send/enableVirtualStick"
EP_ABORT_MISSION = "/send/abortMission"
EP_ABORT_ALL = "/send/abortAll"
EP_GOTO_WP = "/send/gotoWP"
EP_GOTO_YAW = "/send/gotoYaw"
EP_GOTO_WP_PID = "/send/gotoWPwithPID"
EP_GOTO_TRAJECTORY = "/send/navigateTrajectory"
EP_GOTO_ALTITUDE = "/send/gotoAltitude"
EP_CAMERA_START_RECORDING = "/send/camera/startRecording"
EP_CAMERA_STOP_RECORDING = "/send/camera/stopRecording"
EP_GOTO_TRAJECTORY_DJI_NATIVE = "/send/navigateTrajectoryDJINative"
EP_ABORT_DJI_NATIVE_MISSION = "/send/abort/DJIMission"
EP_SET_RTH_ALTITUDE = "/send/setRTHAltitude"
EP_DEACTIVATE_MANUAL_OVERRIDE = "/send/deactivateManualOverride"
EP_GET_MANUAL_OVERRIDE = "/get/isManualOverrideActive"
EP_TUNING = "/send/gotoWPwithPIDtuning"

DiscoveryResult = str | tuple[str | None, str | None] | None


def telemetry_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S.%f")


def get_config(ip_address: str) -> dict[str, Any] | None:
    """Query drone configuration via HTTP GET /config endpoint."""
    try:
        response = requests.get(f"http://{ip_address}:8080/config", timeout=2.0)
        if response.status_code == 200:
            return json.loads(response.text)
    except Exception as exc:
        print(f"Failed to get config from {ip_address}: {exc}")
    return None


def discover_drone(timeout=5.0) -> str | None:
    """Discover the first WildBridge drone on the local network using UDP broadcast."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(timeout)

    try:
        sock.sendto(DISCOVERY_MSG, ("<broadcast>", DISCOVERY_PORT))
        print(f"Broadcasting discovery message on port {DISCOVERY_PORT}...")

        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, addr = sock.recvfrom(1024)
            except TimeoutError:
                break
            discovery_response = parse_discovery_response(data, fallback_ip=addr[0])
            if discovery_response:
                print(f"Found WildBridge drone at {discovery_response.ip_address}")
                return discovery_response.ip_address
    except Exception as exc:
        print(f"Discovery error: {exc}")
    finally:
        sock.close()

    return None


def _normalize_discovery_result(result: DiscoveryResult) -> tuple[str, str]:
    if result is None:
        return "", "UNKNOWN"
    if isinstance(result, tuple):
        ip_address, drone_name = result
        return ip_address or "", drone_name or "UNKNOWN"
    return result, "UNKNOWN"


class DJIInterface:
    """Interface for DJI drone control via HTTP commands and TCP telemetry."""

    def __init__(
        self,
        IP_RC: str = "",
        *,
        discover_callback: Callable[[], DiscoveryResult] | None = None,
        config_loader: Callable[[str], dict[str, Any] | None] | None = None,
        query_config_name: bool = False,
        timestamp_factory: Callable[[], str] = telemetry_timestamp,
    ):
        self.drone_name = "UNKNOWN"
        self._timestamp_factory = timestamp_factory
        config_loader = config_loader or get_config

        if not IP_RC and discover_callback is not None:
            print("No IP provided, attempting to discover drone...")
            discovered_ip, discovered_name = _normalize_discovery_result(discover_callback())
            if discovered_ip:
                self.IP_RC = discovered_ip
                self.drone_name = discovered_name
            else:
                print("Drone discovery failed.")
                self.IP_RC = ""
        else:
            self.IP_RC = IP_RC

        if self.IP_RC and query_config_name:
            config = config_loader(self.IP_RC)
            if config and "droneName" in config:
                self.drone_name = str(config["droneName"])
                print(f"Retrieved drone name from config: {self.drone_name}")

        self.baseCommandUrl = f"http://{self.IP_RC}:8080"
        self.telemetryPort = 8081
        self.videoSource = f"rtsp://aaa:aaa@{self.IP_RC}:8554/streaming/live/1"

        self._telemetry: dict[str, Any] = {}
        self._telemetry_lock = threading.Lock()
        self._telemetry_socket = None
        self._telemetry_thread = None
        self._running = False

    def getVideoSource(self):
        if self.IP_RC == "":
            return ""
        return self.videoSource

    def startTelemetryStream(self):
        """Start receiving telemetry data via TCP socket connection."""
        if self._running:
            return

        self._running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_receiver, daemon=True)
        self._telemetry_thread.start()

    def stopTelemetryStream(self):
        """Stop the telemetry stream and close the socket."""
        self._running = False
        if self._telemetry_socket:
            self._close_telemetry_socket()
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=2)

    def _connect_telemetry_socket(self):
        self._telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._telemetry_socket.settimeout(5.0)
        self._telemetry_socket.connect((self.IP_RC, self.telemetryPort))

    def _process_telemetry_data(self, buffer, data):
        buffer, telemetry_items = parse_telemetry_chunk(
            buffer,
            data,
            timestamp_factory=self._timestamp_factory,
        )
        for telemetry in telemetry_items:
            with self._telemetry_lock:
                self._telemetry = telemetry
        return buffer

    def _read_telemetry_stream(self, buffer):
        while self._running:
            data = self._telemetry_socket.recv(4096)
            if not data:
                break
            buffer = self._process_telemetry_data(buffer, data)
        return buffer

    def _close_telemetry_socket(self):
        with suppress(OSError):
            self._telemetry_socket.close()

    def _telemetry_receiver(self):
        """Background thread that receives telemetry data from TCP socket."""
        buffer = ""
        while self._running:
            try:
                self._connect_telemetry_socket()
                buffer = self._read_telemetry_stream(buffer)
            except TimeoutError:
                continue
            except Exception as exc:
                print(f"Telemetry connection error: {exc}")
                time.sleep(1)
            finally:
                if self._telemetry_socket:
                    self._close_telemetry_socket()

    def getTelemetry(self):
        """Get the latest telemetry data."""
        with self._telemetry_lock:
            return self._telemetry.copy()

    def requestAllStates(self, verbose=False):
        """Get all aircraft states from telemetry."""
        telemetry = self.getTelemetry()
        if verbose and telemetry:
            print("Telemetry:", json.dumps(telemetry, indent=2))
        return telemetry

    def getSpeed(self):
        return self.getTelemetry().get("speed", {})

    def getHeading(self):
        return self.getTelemetry().get("heading", 0.0)

    def getAttitude(self):
        return self.getTelemetry().get("attitude", {})

    def getLocation(self):
        return self.getTelemetry().get("location", {})

    def getGimbalAttitude(self):
        return self.getTelemetry().get("gimbalAttitude", {})

    def getGimbalJointAttitude(self):
        return self.getTelemetry().get("gimbalJointAttitude", {})

    def getZoomFocalLength(self):
        return self.getTelemetry().get("zoomFl", -1)

    def getHybridFocalLength(self):
        return self.getTelemetry().get("hybridFl", -1)

    def getOpticalFocalLength(self):
        return self.getTelemetry().get("opticalFl", -1)

    def getZoomRatio(self):
        return self.getTelemetry().get("zoomRatio", 1.0)

    def getBatteryLevel(self):
        return self.getTelemetry().get("batteryLevel", -1)

    def getSatelliteCount(self):
        return self.getTelemetry().get("satelliteCount", -1)

    def getHomeLocation(self):
        return self.getTelemetry().get("homeLocation", {})

    def getDistanceToHome(self):
        return self.getTelemetry().get("distanceToHome", 0.0)

    def isWaypointReached(self):
        return self.getTelemetry().get("waypointReached", False)

    def isIntermediaryWaypointReached(self):
        return self.getTelemetry().get("intermediaryWaypointReached", False)

    def isYawReached(self):
        return self.getTelemetry().get("yawReached", False)

    def isAltitudeReached(self):
        return self.getTelemetry().get("altitudeReached", False)

    def isCameraRecording(self):
        return self.getTelemetry().get("isRecording", False)

    def isHomeSet(self):
        return self.getTelemetry().get("homeSet", False)

    def getRemainingFlightTime(self):
        return self.getTelemetry().get("remainingFlightTime", 0)

    def getTimeNeededToGoHome(self):
        return self.getTelemetry().get("timeNeededToGoHome", 0)

    def getTimeNeededToLand(self):
        return self.getTelemetry().get("timeNeededToLand", 0)

    def getTotalTime(self):
        return self.getTelemetry().get("totalTime", 0)

    def getMaxRadiusCanFlyAndGoHome(self):
        return self.getTelemetry().get("maxRadiusCanFlyAndGoHome", 0)

    def getRemainingCharge(self):
        return self.getTelemetry().get("remainingCharge", 0)

    def getBatteryNeededToLand(self):
        return self.getTelemetry().get("batteryNeededToLand", 0)

    def getBatteryNeededToGoHome(self):
        return self.getTelemetry().get("batteryNeededToGoHome", 0)

    def getSeriousLowBatteryThreshold(self):
        return self.getTelemetry().get("seriousLowBatteryThreshold", 0)

    def getLowBatteryThreshold(self):
        return self.getTelemetry().get("lowBatteryThreshold", 0)

    def getFlightMode(self):
        return self.getTelemetry().get("flightMode", "UNKNOWN")

    def isManualOverrideActive(self):
        return self.getTelemetry().get("isManualOverrideActive", False)

    def requestSend(self, endPoint, data, verbose=False):
        """Send a POST request to the drone."""
        if self.IP_RC == "":
            print(f"No IP_RC provided, returning empty string for request at {endPoint}")
            return ""
        try:
            response = requests.post(
                build_command_url(self.baseCommandUrl, endPoint), str(data), timeout=5
            )
            if verbose:
                print("EP : " + endPoint + "\t" + str(response.content, encoding="utf-8"))
            return response.content.decode("utf-8")
        except requests.exceptions.RequestException as exc:
            print(f"Request error at {endPoint}: {exc}")
            return ""

    def requestSendStick(self, leftX=0, leftY=0, rightX=0, rightY=0):
        s = 0.3
        leftX = max(-s, min(s, leftX))
        leftY = max(-s, min(s, leftY))
        rightX = max(-s, min(s, rightX))
        rightY = max(-s, min(s, rightY))
        return self.requestSend(EP_STICK, f"{leftX:.4f},{leftY:.4f},{rightX:.4f},{rightY:.4f}")

    def requestSendGimbalPitch(self, pitch=0):
        return self.requestSend(EP_GIMBAL_SET_PITCH, f"0,{pitch},0")

    def requestSendGimbalYaw(self, yaw=0):
        return self.requestSend(EP_GIMBAL_SET_YAW, f"0,0,{yaw}")

    def requestSendZoomRatio(self, zoomRatio=1):
        return self.requestSend(EP_ZOOM, zoomRatio)

    def requestSendTakeOff(self):
        return self.requestSend(EP_TAKEOFF, "")

    def requestSendLand(self):
        return self.requestSend(EP_LAND, "")

    def requestSendRTH(self):
        self.requestAbortMission()
        return self.requestSend(EP_RTH, "")

    def requestSendGoToWP(self, latitude, longitude, altitude):
        return self.requestSend(EP_GOTO_WP, f"{latitude},{longitude},{altitude}")

    def requestSendGoToWPwithPID(self, latitude, longitude, altitude, yaw, speed: float = 5.0):
        return self.requestSend(EP_GOTO_WP_PID, f"{latitude},{longitude},{altitude},{yaw},{speed}")

    def requestSendGoToWPwithPIDtuning(
        self, latitude, longitude, altitude, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw
    ):
        return self.requestSend(
            EP_TUNING,
            f"{latitude},{longitude},{altitude},{yaw},{kp_pos},{ki_pos},{kd_pos},{kp_yaw},{ki_yaw},{kd_yaw}",
        )

    def requestSendNavigateTrajectory(self, waypoints, finalYaw):
        self.requestSendEnableVirtualStick()
        if not waypoints:
            raise ValueError("No waypoints provided")

        segments = []
        for index, (lat, lon, alt) in enumerate(waypoints):
            if index < len(waypoints) - 1:
                segments.append(f"{lat},{lon},{alt}")
            else:
                segments.append(f"{lat},{lon},{alt},{finalYaw}")

        return self.requestSend(EP_GOTO_TRAJECTORY, ";".join(segments))

    def requestSendNavigateTrajectoryDJINative(self, waypoints, speed: float = 10.0):
        if not waypoints:
            raise ValueError("No waypoints provided")
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints for DJI native mission")

        segments = [str(speed)]
        for lat, lon, alt in waypoints:
            segments.append(f"{lat},{lon},{alt}")

        return self.requestSend(EP_GOTO_TRAJECTORY_DJI_NATIVE, ";".join(segments))

    def requestAbortDJINativeMission(self):
        return self.requestSend(EP_ABORT_DJI_NATIVE_MISSION, "")

    def requestAbortMission(self):
        return self.requestSend(EP_ABORT_MISSION, "")

    def requestAbortAll(self):
        return self.requestSend(EP_ABORT_ALL, "")

    def requestSendEnableVirtualStick(self):
        return self.requestSend(EP_ENABLE_VIRTUAL_STICK, "")

    def requestSendGotoYaw(self, yaw):
        self.requestSendEnableVirtualStick()
        return self.requestSend(EP_GOTO_YAW, f"{yaw}")

    def requestSendGotoAltitude(self, altitude):
        self.requestSendEnableVirtualStick()
        return self.requestSend(EP_GOTO_ALTITUDE, f"{altitude}")

    def requestCameraStartRecording(self):
        return self.requestSend(EP_CAMERA_START_RECORDING, "")

    def requestCameraStopRecording(self):
        return self.requestSend(EP_CAMERA_STOP_RECORDING, "")

    def requestSetRTHAltitude(self, altitude):
        return self.requestSend(EP_SET_RTH_ALTITUDE, str(altitude))

    def requestDeactivateManualOverride(self):
        return self.requestSend(EP_DEACTIVATE_MANUAL_OVERRIDE, "")

    def requestSticks(self):
        print("Warning: requestSticks() is deprecated. Use getTelemetry() instead.")
        return ""

    def requestWaypointStatus(self):
        return str(self.isWaypointReached()).lower()

    def requestIntermediaryWaypointStatus(self):
        return str(self.isIntermediaryWaypointReached()).lower()

    def requestYawStatus(self):
        return str(self.isYawReached()).lower()

    def requestAltitudeStatus(self):
        return str(self.isAltitudeReached()).lower()

    def requestHomePosition(self):
        return self.getHomeLocation()

    def requestCameraIsRecording(self):
        return self.isCameraRecording()
