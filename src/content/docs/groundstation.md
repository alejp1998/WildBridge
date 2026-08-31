---
title: Ground Station
description: The Python DJIInterface client and the multi-drone video dashboard.
breadcrumb: Start here
---

## Python interface (`DJIInterface`)

`GroundStation/Python/wildbridge_groundstation/dji_client.py` provides a high-level class wrapping all HTTP commands and the TCP telemetry socket in a thread-safe background receiver.

```python
import time
from wildbridge_groundstation.dji_client import DJIInterface, discover_drone

# Auto-discovery via UDP broadcast (port 30000) if no IP provided
dji = DJIInterface("", discover_callback=discover_drone)

# Start background telemetry thread (TCP socket, port 8081)
dji.startTelemetryStream()

# Read latest telemetry (thread-safe, returns copy of last JSON snapshot)
print(dji.getBatteryLevel())          # int: 0–100
print(dji.getLocation())              # {'latitude': ..., 'longitude': ..., 'altitude': ...}
print(dji.getHeading())               # float: compass degrees
print(dji.getAttitude())              # {'pitch': ..., 'roll': ..., 'yaw': ...}
print(dji.getGimbalAttitude())        # {'pitch': ..., 'roll': ..., 'yaw': ...}
print(dji.getSatelliteCount())        # int
print(dji.getFlightMode())            # str: 'GPS', 'ATTI', 'VIRTUAL_STICK', 'GO_HOME', ...
print(dji.isManualOverrideActive())   # bool
print(dji.getRemainingFlightTime())   # int: seconds
print(dji.getDistanceToHome())        # float: metres
print(dji.getZoomRatio())             # float

# Commands
dji.requestSendTakeOff()
dji.requestSendLand()
dji.requestSendRTH()                  # Aborts mission first, then RTH
dji.requestSendEnableVirtualStick()
dji.requestAbortMission()             # Abort + disable Virtual Stick
dji.requestAbortDJINativeMission()    # Abort DJI native mission only

# Navigation
# Nose-forward: drone turns to face the leg, flies forward, then rotates to yaw on arrival.
dji.requestSendGoToWaypointNoseForward(49.306254, 4.593728, 20.0, yaw=90, speed=5.0)
# Hold-heading: nose stays on yaw for the whole flight (drone crabs sideways), tighter tolerance.
dji.requestSendGoToWaypointHoldHeading(49.306254, 4.593728, 20.0, yaw=90, speed=5.0)
dji.requestSendNavigateTrajectoryDJINative(
    [(49.306, 4.593, 20), (49.307, 4.594, 25), (49.308, 4.595, 20)], speed=10.0)
dji.requestSendGotoYaw(45.0)
dji.requestSendGotoAltitude(30.0)

# Camera / gimbal
dji.requestSendGimbalPitch(-30.0)
dji.requestSendGimbalYaw(45.0)
dji.requestSendGimbalRelPitch(-5.0)     # relative to the current angle
dji.requestSendGimbalRelYaw(10.0)
dji.requestSendZoomRatio(4.0)
dji.requestCameraStartRecording()
dji.requestCameraStopRecording()

# Thermal / payload
dji.requestCapture()                     # capture descriptor for the thermal lens
dji.requestCaptureTemperature()          # thermal max temperature
dji.requestLRFMeasure()                  # distance + geo-referenced target + laser state
dji.getLRFTarget()                       # last locked target from telemetry
dji.requestDrop()                        # release payload (airframes with a drop port)

# Media
files = dji.listMedia()                  # [{"name", "index", "size", "type"}, ...]
dji.downloadByName(files[0]["name"], out_dir="./media")

# Sequence-tracked commands — avoids acting on a stale reach flag
seq = dji.requestSendGoToWaypointNoseForward(49.306254, 4.593728, 20.0, yaw=90, speed=5.0)
while not dji.isWaypointReached(seq):
    time.sleep(0.1)

# Preflight
dji.isReadyToTakeoff()
dji.getTakeoffBlockReason()

# Manual override
dji.requestDeactivateManualOverride()

# RTH altitude
dji.requestSetRTHAltitude(50.0)

# Virtual stick (raw AVS, values saturated to ±0.3 by DJIInterface)
dji.requestSendStick(leftX=0, leftY=0.2, rightX=0.1, rightY=0)

dji.stopTelemetryStream()
```

The two-computer safety wrapper lives in `wildbridge_groundstation.safety` — see [HTTP API](/http-api/#pilot-safety-authority).

## GroundStation video dashboard

The video-test stack runs MediaMTX plus a browser dashboard for multi-drone video testing and stream diagnostics. It discovers phones, connects to telemetry on port 8081, displays stream health, and consumes video through WHEP.

Default services:

| Service | Default URL / Port |
|---------|--------------------|
| Browser dashboard | http://localhost:8090 |
| MediaMTX WebRTC / WHIP / WHEP | http://localhost:8889 |
| MediaMTX API | http://localhost:9997 |
| MediaMTX RTSP | rtsp://localhost:8554 |
| ICE UDP | :8189 |

Useful restart command:

```bash
docker compose -f GroundStation/video_test/compose.yaml down
docker compose -f GroundStation/video_test/compose.yaml up -d
docker compose -f GroundStation/video_test/compose.yaml ps
```

Runtime diagnostics are written under `GroundStation/video_test/logs/`. Those logs are intentionally ignored by git.

Settings can be viewed and changed for each drone from the **Settings** tab: it reads and writes DJI flight limits (RTH altitude, max height/distance), RC pairing and stick mode, and app/video settings over HTTP, and shows read-only rows for the drone name, the detected aircraft model, and the control profile (speed/PID/gimbal profile) automatically selected for it — so you can confirm the right profile is active without opening the app on the phone.

| Tab | What it shows |
|-----|---------------|
| **Video** | Live WHIP/WHEP tiles per drone with quick FPS/loss/telemetry stats |
| **Health** | Correlated phone/sender/MediaMTX/browser diagnostics, worst symptom first |
| **Video Charts** | Decoded FPS, bitrate, packet loss, and jitter over time |
| **Telemetry** | Full nested live state tree per drone |
| **Telemetry Charts** | Battery, satellites, altitude, and Wi-Fi RSSI over time |
| **Settings** | View and change DJI/app settings per drone over HTTP |
| **HTTP** | The full `/send/` catalogue, each entry marked if it also has a MAVLink form |
| **MAVLink** | What the aircraft reports and accepts as a MAVLink 2 vehicle, with an HTTP cross-check |
| **ROS** | Per-drone ROS topic liveness and rates from ros-monitor |

![Video dashboard](../../../docs/images/VideoTestTab.png)
