---
title: ROS 2 Integration
description: The wildbridge_controller node publishes 45+ topics per drone and subscribes to command topics under the drone's own namespace.
breadcrumb: Interfaces
---

Full ROS 2 Humble package. The `wildbridge_controller` node publishes all telemetry fields as individual topics and subscribes to command topics. It checks for new telemetry at 20 Hz but publishes only when the drone has actually sent a new snapshot, so the topic rate follows the aircraft's telemetry interval (default ~2 Hz) rather than repeating each sample.

## Package structure

```text
GroundStation/ROS/
├── wildbridge_controller/   # Main control + telemetry node
│   ├── controller.py        # DjiNode: 45+ topics, publishes on new telemetry
│   └── dji_interface.py
├── wildbridge_videofeed/    # RTSP video feed -> sensor_msgs/Image
└── wildbridge_bringup/
    ├── auto_discovery_native.launch.py  # one namespaced wildbridge_controller per discovered drone, re-scans periodically
    ├── swarm_connection.launch.py
    └── config/parameters.yaml
```

## Published topics (per drone namespace, e.g. `/mini1/`)

| Topic | Type | Description |
|-------|------|-------------|
| `speed` | `Float64` | Velocity magnitude (m/s) |
| `speed_vector` | `Vector3` | Velocity vector x, y, z (m/s) |
| `heading` | `Float64` | Compass heading (degrees) |
| `attitude` | `String` | `{pitch, roll, yaw}` JSON |
| `location` | `NavSatFix` | GPS latitude, longitude, altitude |
| `gimbal_attitude` | `String` | `{pitch, roll, yaw}` JSON |
| `gimbal_joint_attitude` | `String` | Joint angles JSON |
| `gimbal_yaw` / `gimbal_pitch` | `Float64` | Individual gimbal angles |
| `zoom_fl` / `hybrid_fl` / `optical_fl` | `Float64` | Focal lengths (-1 if N/A) |
| `zoom_ratio` | `Float64` | Camera zoom ratio |
| `battery_level` | `Float64` | Battery % |
| `satellite_count` | `Int32` | GPS satellite count |
| `waypoint_reached` | `Bool` | Final WP flag |
| `intermediary_waypoint_reached` | `Bool` | Intermediate WP flag |
| `altitude_reached` / `yaw_reached` | `Bool` | Reach flags |
| `home_location` | `NavSatFix` | Home GPS coordinates |
| `home_set` | `Bool` | Home point set flag |
| `distance_to_home` | `Float64` | Distance to home (m) |
| `remaining_flight_time` | `Float64` | Remaining flight time (s) |
| `time_needed_to_go_home` | `Float64` | Time to RTH (s) |
| `time_needed_to_land` | `Float64` | Time to land (s) |
| `time_to_landing_spot` | `Float64` | Go-home + land (s) |
| `max_radius_can_fly_and_go_home` | `Float64` | Max safe radius (m) |
| `battery_needed_to_go_home` | `Float64` | Battery % for RTH |
| `battery_needed_to_land` | `Float64` | Battery % to land |
| `camera/is_recording` | `Bool` | Recording status |
| `flight_mode` | `String` | Current DJI flight mode string |
| `manual_override_active` | `Bool` | Pilot override active |
| `waypoint_seq` / `yaw_seq` / `altitude_seq` | `Int32` | Sequence id of the command currently executing |
| `command_ack/waypoint_seq` / `yaw_seq` / `altitude_seq` | `Int32` | Sequence id assigned when the command was accepted |
| `ready_to_takeoff` | `Bool` | Preflight conditions satisfied |
| `takeoff_block_reason` | `String` | Why takeoff is blocked |
| `camera/capture_result` | `String` | Capture descriptor JSON (async) |
| `camera/media_list` | `String` | SD card listing JSON (async) |
| `camera/download_result` | `String` | Download result JSON (async) |
| `camera/thermal_max_temp` | `Float64` | Thermal max temperature (°C) |
| `lrf/measurement` | `String` | Laser rangefinder reading JSON |
| `lrf/target` | `NavSatFix` | Geo-referenced laser target |

## Subscribed topics (commands)

Camera, media, and LRF commands block for as long as the aircraft takes to answer (up to 120 s for
a download), so they run on a single-worker thread pool and answer asynchronously on their own
`camera/*` and `lrf/*` result topics rather than stalling the telemetry loop.

| Topic | Type | Body |
|-------|------|------|
| `command/takeoff` | `Empty` | — |
| `command/land` | `Empty` | — |
| `command/rth` | `Empty` | — |
| `command/abort_mission` | `Empty` | — |
| `command/abort_all` | `Empty` | — |
| `command/enable_virtual_stick` | `Empty` | — |
| `command/abort_dji_native_mission` | `Empty` | — |
| `command/deactivate_manual_override` | `Empty` | — |
| `command/camera/start_recording` | `Empty` | — |
| `command/camera/stop_recording` | `Empty` | — |
| `command/goto_waypoint_nose_forward` | `Float64MultiArray` | `[lat, lon, alt, yaw, speed?]` — `yaw` = final arrival heading |
| `command/goto_waypoint_hold_heading` | `Float64MultiArray` | `[lat, lon, alt, yaw, speed?]` — `yaw` held for the whole flight |
| `command/goto_trajectory_dji_native` | `String` | `"(speed, [(lat,lon,alt),...])"` |
| `command/goto_yaw` | `Float64` | Yaw angle (degrees) |
| `command/goto_altitude` | `Float64` | Altitude (m) |
| `command/gimbal_pitch` | `Float64` | Pitch (degrees) |
| `command/gimbal_yaw` | `Float64` | Yaw joint angle |
| `command/zoom_ratio` | `Float64` | Zoom ratio |
| `command/set_rth_altitude` | `Float64` | RTH altitude (m) |
| `command/stick` | `Float64MultiArray` | `[leftX, leftY, rightX, rightY]` ∈ [-1,1] |
| `command/gimbal_rel_pitch` | `Float64` | Pitch relative to current angle (degrees) |
| `command/gimbal_rel_yaw` | `Float64` | Yaw relative to current angle (degrees) |
| `command/camera/capture` | `Empty` | Capture on the thermal lens |
| `command/camera/capture_temperature` | `Empty` | Read thermal max temperature |
| `command/camera/list_media` | `Empty` | List SD card contents |
| `command/camera/download_media` | `String` | Download one media file by name |
| `command/lrf/measure` | `Empty` | Fire the laser rangefinder |
| `command/drop` | `Empty` | Release the payload |

## Usage

**Docker (single-drone, auto-discovery):**

```bash
cd GroundStation
docker build -t wildbridge-ros .
docker run --rm --network=host wildbridge-ros
```

The image is based on `ros:humble` with CycloneDDS, `cv-bridge`, `vision-opencv`, `image-transport`, plus all Python dependencies.

**Manual multi-drone launch:**

```bash
cd GroundStation/ROS
colcon build --symlink-install && source install/setup.bash
ros2 launch wildbridge_bringup auto_discovery_native.launch.py

# Example commands (namespace is the drone's own name, e.g. "mini1")
ros2 topic pub /mini1/command/takeoff std_msgs/msg/Empty "{}"
ros2 topic pub /mini1/command/goto_waypoint_nose_forward std_msgs/msg/Float64MultiArray \
  "data: [49.306254, 4.593728, 20.0, 90.0, 5.0]"
```
