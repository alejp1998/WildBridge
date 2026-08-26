#!/usr/bin/env python3
"""WildBridge ROS topic monitor.

Watches the live ROS graph for `dji_node_*` nodes (one per drone, namespaced
under its name — see launch_controllers.py and
GroundStation/ROS/wildview_bringup/launch/auto_discovery_native.launch.py),
subscribes to each drone's topics under its own namespace, tracks per-topic
publish metrics (rate, last value, freshness) per drone, probes each drone's
phone HTTP surface, and reports the result to the WildBridge webapp via
POST /api/ros-status so the dashboard can show per-drone ROS bridge health.
"""

import json
import os
import time
import urllib.request
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, Int32, String

WEBAPP_URL = os.environ.get("WEBAPP_URL", "http://127.0.0.1:8090")

ALLOWED_URL_SCHEMES = ("http", "https")


def _open_url(target, timeout):
    """urlopen restricted to HTTP(S).

    Every URL here is built from an environment variable, so whoever sets the environment could
    otherwise point these calls at a file:// path or a custom scheme and have the process read
    local files. Validating the scheme is what makes the call safe; the nosec records that it was
    checked rather than ignored, and keeping the check in one place means a new call site cannot
    quietly skip it.
    """
    url = target.full_url if isinstance(target, urllib.request.Request) else target
    scheme = urlparse(url).scheme
    if scheme not in ALLOWED_URL_SCHEMES:
        raise ValueError(f"refusing to open URL with scheme {scheme!r}: {url!r}")
    return urllib.request.urlopen(target, timeout=timeout)  # nosec B310 - scheme checked above


REPORT_INTERVAL = float(os.environ.get("ROS_REPORT_INTERVAL", "3"))
SYNC_INTERVAL = float(os.environ.get("ROS_SYNC_INTERVAL", "2"))
# How often to retry the phone HTTP probe for a drone that isn't reachable yet
# (e.g. its ROS node appeared before the webapp resolved its IP). Reachable
# drones are never re-probed.
PHONE_RETRY_INTERVAL = 10.0

# Topic name -> message type, mirroring DjiNode's publishers in controller.py.
# Subscribed per-drone as f"/{namespace}/{topic}".
PUBLISHED_TOPICS = {
    "state/settings": String,
    "speed": Float64,
    "speed_vector": Vector3,
    "heading": Float64,
    "attitude": String,
    "location": NavSatFix,
    "gimbal_attitude": String,
    "gimbal_joint_attitude": String,
    "zoom_fl": Float64,
    "hybrid_fl": Float64,
    "optical_fl": Float64,
    "zoom_ratio": Float64,
    "battery_level": Float64,
    "satellite_count": Int32,
    "gimbal_yaw": Float64,
    "gimbal_pitch": Float64,
    "waypoint_reached": Bool,
    "intermediary_waypoint_reached": Bool,
    "altitude_reached": Bool,
    "yaw_reached": Bool,
    "home_location": NavSatFix,
    "home_set": Bool,
    "distance_to_home": Float64,
    "remaining_flight_time": Float64,
    "time_needed_to_go_home": Float64,
    "time_needed_to_land": Float64,
    "time_to_landing_spot": Float64,
    "max_radius_can_fly_and_go_home": Float64,
    "battery_needed_to_go_home": Float64,
    "battery_needed_to_land": Float64,
    "camera/is_recording": Bool,
    "flight_mode": String,
    "manual_override_active": Bool,
    "waypoint_seq": Int32,
    "yaw_seq": Int32,
    "altitude_seq": Int32,
    "command_ack/waypoint_seq": Int32,
    "command_ack/yaw_seq": Int32,
    "command_ack/altitude_seq": Int32,
    "lrf/target": NavSatFix,
    "lrf/measurement": String,
    "camera/thermal_max_temp": Float64,
    "camera/capture_result": String,
    "camera/media_list": String,
    "camera/download_result": String,
    "ready_to_takeoff": Bool,
    "takeoff_block_reason": String,
}

# Command topics DjiNode subscribes to (the ROS control surface). The monitor
# subscribes to these too so it can report when commands are actually sent.
COMMAND_TOPICS = {
    "command/takeoff": Empty,
    "command/land": Empty,
    "command/rth": Empty,
    "command/abort_mission": Empty,
    "command/abort_all": Empty,
    "command/enable_virtual_stick": Empty,
    "command/abort_dji_native_mission": Empty,
    "command/deactivate_manual_override": Empty,
    "command/goto_waypoint_nose_forward": Float64MultiArray,
    "command/goto_trajectory_dji_native": String,
    "command/goto_yaw": Float64,
    "command/goto_altitude": Float64,
    "command/gimbal_pitch": Float64,
    "command/gimbal_yaw": Float64,
    "command/zoom_ratio": Float64,
    "command/set_rth_altitude": Float64,
    "command/set_setting": String,
    "command/stick": Float64MultiArray,
    "command/gimbal_rel_pitch": Float64,
    "command/gimbal_rel_yaw": Float64,
    "command/goto_waypoint_hold_heading": Float64MultiArray,
    "command/camera/start_recording": Empty,
    "command/camera/stop_recording": Empty,
    "command/camera/capture": Empty,
    "command/camera/capture_temperature": Empty,
    "command/camera/list_media": Empty,
    "command/camera/download_media": String,
    "command/lrf/measure": Empty,
    "command/drop": Empty,
}

TOPICS = {**PUBLISHED_TOPICS, **COMMAND_TOPICS}


def _format_value(msg) -> str:
    if hasattr(msg, "data"):
        return str(msg.data)
    if hasattr(msg, "latitude"):
        return f"lat={msg.latitude:.6f} lon={msg.longitude:.6f}"
    if hasattr(msg, "x"):
        return f"({msg.x}, {msg.y}, {msg.z})"
    return str(msg)


def _fetch_drone_ips():
    """Best-effort name -> ip lookup from the webapp's own discovery state, used
    only to probe each drone's phone HTTP surface (not required for ROS)."""
    try:
        with _open_url(f"{WEBAPP_URL}/api/drones", 3) as resp:
            state = json.loads(resp.read().decode("utf-8"))
        return {
            drone["name"]: drone.get("ip") for drone in state.get("drones", []) if drone.get("ip")
        }
    except Exception:
        return {}


def _probe_phone(ip):
    if not ip:
        return False, "no ip known for this drone yet"
    try:
        with _open_url(f"http://{ip}:8080/config", 3) as resp:
            return resp.status == 200, ""
    except Exception as exc:
        return False, str(exc)[:120]


class RosMonitor(Node):
    def __init__(self):
        super().__init__("ros_monitor")
        # namespace -> {topic: {count, last_value, last_time, type}}
        self.drone_stats = {}
        # namespace -> {topic: Subscription}
        self.drone_subs = {}
        # namespace -> {"reachable": bool, "error": str}
        self.drone_phone = {}
        self.last_reset = time.time()
        self._sync_drones()
        self.create_timer(SYNC_INTERVAL, self._sync_drones)
        self.create_timer(REPORT_INTERVAL, self._report)

    def _sync_drones(self):
        try:
            nodes = self.get_node_names_and_namespaces()
        except Exception:
            return
        active = {
            ns.strip("/") for name, ns in nodes if name.startswith("dji_node") and ns.strip("/")
        }
        for namespace in active - self.drone_stats.keys():
            self._attach_drone(namespace)
        for namespace in self.drone_stats.keys() - active:
            self._detach_drone(namespace)
        self._refresh_phone_reachability(active)

    def _attach_drone(self, namespace):
        self.get_logger().info(f"Attaching drone namespace: {namespace}")
        stats = {}
        subs = {}
        for topic, msg_type in TOPICS.items():
            stats[topic] = {
                "count": 0,
                "last_value": None,
                "last_time": None,
                "type": msg_type.__name__,
            }
            subs[topic] = self.create_subscription(
                msg_type, f"/{namespace}/{topic}", self._make_callback(namespace, topic), 10
            )
        self.drone_stats[namespace] = stats
        self.drone_subs[namespace] = subs
        self.drone_phone[namespace] = {
            "reachable": False,
            "error": "not probed yet",
            "checked_at": 0.0,
        }

    def _detach_drone(self, namespace):
        self.get_logger().info(f"Detaching drone namespace: {namespace}")
        for sub in self.drone_subs.pop(namespace, {}).values():
            self.destroy_subscription(sub)
        self.drone_stats.pop(namespace, None)
        self.drone_phone.pop(namespace, None)

    def _refresh_phone_reachability(self, namespaces):
        now = time.time()
        due = [
            ns
            for ns in namespaces
            if not self.drone_phone.get(ns, {}).get("reachable")
            and now - self.drone_phone.get(ns, {}).get("checked_at", 0.0) >= PHONE_RETRY_INTERVAL
        ]
        if not due:
            return
        ip_by_name = _fetch_drone_ips()
        for namespace in due:
            reachable, error = _probe_phone(ip_by_name.get(namespace))
            self.drone_phone[namespace] = {
                "reachable": reachable,
                "error": error,
                "checked_at": now,
            }

    def _make_callback(self, namespace, topic):
        def callback(msg):
            entry = self.drone_stats.get(namespace, {}).get(topic)
            if entry is None:
                return
            entry["count"] += 1
            entry["last_value"] = _format_value(msg)[:120]
            entry["last_time"] = time.time()

        return callback

    def _report(self):
        now = time.time()
        elapsed = max(now - self.last_reset, 1e-6)
        self.last_reset = now

        drones = {}
        for namespace, stats in self.drone_stats.items():
            topics = {}
            for topic, entry in stats.items():
                rate = entry["count"] / elapsed if entry["count"] else 0.0
                topics[topic] = {
                    "type": entry["type"],
                    "rate_hz": round(rate, 2),
                    "last_value": entry["last_value"],
                    "seconds_ago": (
                        round(now - entry["last_time"], 1) if entry["last_time"] else None
                    ),
                    "seen": entry["count"] > 0,
                }
                entry["count"] = 0
            phone = self.drone_phone.get(namespace, {"reachable": False, "error": ""})
            drones[namespace] = {
                "controllerAlive": True,
                "phoneReachable": phone["reachable"],
                "phoneError": phone["error"],
                "topicCount": len(topics),
                "topics": topics,
            }

        payload = {
            "generatedAt": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "droneCount": len(drones),
            "drones": drones,
        }
        try:
            req = urllib.request.Request(
                f"{WEBAPP_URL}/api/ros-status",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with _open_url(req, 3):
                pass
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RosMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
