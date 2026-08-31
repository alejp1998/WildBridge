---
title: MAVLink 2 Interface
description: WildBridge presents each aircraft as a MAVLink 2 vehicle on UDP port 14550, so QGroundControl, MAVSDK, and pymavlink fly it with no plugin.
breadcrumb: Interfaces
---

WildBridge presents each aircraft as a **MAVLink 2 vehicle**, so a stock ground station — QGroundControl, MAVSDK, `pymavlink` — connects, sees telemetry and video, and flies it with no plugin and no configuration file.

This is a full control surface, not a telemetry feed. Every command the ROS ground station uses has a MAVLink form, and so does every `/send/set*` setting. The HTTP surface on port 8080 still exists and is unchanged; the two are kept in step deliberately, and the dashboard's **MAVLink** tab reads both at once so a disagreement between them is visible rather than inferred.

**Disabled by default.** Following PX4's pattern of switching MAVLink instances on by parameter rather than by build:

| Preference | Type | Default | Meaning |
|------------|------|---------|---------|
| `wb_mav_0_enabled` | bool | `false` | Start the endpoint at all |
| `wb_mav_0_host` | string | *(empty)* | Ground-station address. Empty means broadcast on the subnet |
| `wb_mav_0_port` | int | `14550` | Port to send to and listen on |
| `wb_mav_0_mode` | string | `normal` | Stream profile: `normal` or `minimal` |
| `wb_mav_0_sysid` | int | `1` | MAVLink system id — one per aircraft |
| `wb_mav_0_allow_flight` | bool | `false` | Allow commands that move the aircraft |
| `wb_mav_0_signing_key` | string | *(empty)* | 64 hex characters shared with the Safety Computer |
| `wb_mission_exec` | string | `onboard` | Who flies an uploaded plan: `onboard` or `dji_native` |

Flight commands are gated twice: `wb_mav_0_allow_flight` must be on, and the command must pass the same `ControlAuthority` check the HTTP surface uses. Anything that would fly the aircraft somewhere new is refused while the pilot holds the sticks; land, return and abort stay available, because those are the recovery actions.

## Protocols implemented

| Microservice | What works |
|---|---|
| **Telemetry** | Heartbeat, attitude, position, GPS, battery, VFR HUD, extended system state, home position, current mode, mission progress, gimbal attitude, rangefinder distance |
| **Command** | Take-off (with altitude), land, return, reposition, yaw, altitude, stick input, arm/disarm, camera, gimbal, payload release |
| **Mission** | Upload and download handshake, `MISSION_START`, progress and arrival reports. Per-waypoint hold time, acceptance radius and pass-through honoured |
| **Parameter** | `PARAM_SET` for numeric settings; `PARAM_EXT_SET` for string settings such as the drone name, video source and MediaMTX address |
| **Camera / Gimbal** | Camera information, settings, capture status, video stream information; gimbal attitude and pitch/yaw control |
| **File transfer** | Read-only MAVLink FTP for listing and reading the SD card |
| **Signing** | MAVLink 2 packet signing identifies the Safety Computer, mirroring `X-Safety-Token` on the HTTP surface |

## Two conventions worth knowing

**Heading, per waypoint.** `NAV_WAYPOINT.param4` is `NaN` for "use the vehicle's own heading mode" and a value for "hold this heading". That is exactly the difference between WildBridge's two waypoint controllers, so one plan can mix them, and `DO_REPOSITION` reads it the same way. No custom mission item was needed.

**Arrival, per waypoint.** `param1` (hold time), `param2` (acceptance radius) and `param3` (pass through) come from the plan, because only the plan knows which leg is the last one. A leg marked pass-through is flown through; the final one settles.

## Running more than one ground station

A UDP datagram goes to exactly one socket, so each ground station needs its own listen port; the aircraft fans telemetry out to every station it has heard from.

| | Listens | Sends to aircraft |
|---|---|---|
| QGroundControl | 14550 | 14550 |
| ROS drone nodes (`WB_MAVLINK_PORT`) | 14551 | 14550 |
| Dashboard MAVLink tab (`WB_WEBAPP_MAVLINK_PORT`) | 14552 | 14550 |

A fleet needs one port per aircraft for the same reason.

## Verify without QGroundControl

```bash
pip install pymavlink
wildbridge-mavlink-listen --summary 5
```

It prints the first instance of each message with decoded values, then a rate summary, and reports malformed frames loudly as `BAD_DATA`. It never transmits.

**Connect QGroundControl:** it listens on UDP 14550 by default and adds a link on receiving traffic. If the RC and the ground station share a LAN, enabling `wb_mav_0_enabled` is all that is required.

WildBridge reports `MAV_AUTOPILOT_PX4`. It does not run PX4 — the claim exists because QGroundControl only enables its Fly View action buttons for firmware plugins that declare guided-mode capability, and its generic plugin declares none. Values DJI does not provide use MAVLink's documented "unknown" conventions rather than plausible-looking zeros.

## Field testing

The [Field Test](/field-test/) page is the procedure for verifying all of this against a real aircraft. The ground half runs itself:

```bash
cd GroundStation/Python
python test_scripts/field_check.py <PHONE_IP>                    # listens only
python test_scripts/field_check.py <PHONE_IP> --phase ground     # parameter writes
python test_scripts/field_check.py <PHONE_IP> --phase payload --move
python test_scripts/field_check.py <PHONE_IP> --phase flight --fly
```

Checks are grouped by what they can move, and the script will not cross a group boundary without being told to: it sends nothing at all by default, needs `--move` before the gimbal or camera responds, and only prints the flight list under `--fly`.

## Video streaming

WildBridge's supported public video path is WebRTC publishing through WHIP to MediaMTX, with browser playback through WHEP. The app publishes DJI camera frames to a WHIP URL selected by the ground station, normally:

```text
http://<ground-station-ip>:8889/<drone_name>/whip
```

The GroundStation video dashboard then watches the matching WHEP URL:

```text
http://<ground-station-ip>:8889/<drone_name>/whep
```

This replaces the older direct RC-hosted WebSocket-signaling viewer. That sample viewer/server path is no longer part of the public app or ground-station tooling.
