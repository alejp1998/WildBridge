---
title: Getting Started
description: Install WildBridge on your DJI RC, build it from source, and connect your first ground station.
breadcrumb: Start here
---

This guide takes you from a stock DJI drone and remote controller to a live telemetry stream and your first HTTP command.

## Prerequisites

1. DJI drone + compatible RC, 5 GHz Wi-Fi access point, ground station computer
2. [Android Studio Koala 2024.1.2](https://redirector.gvt1.com/edgedl/android/studio/ide-zips/2024.1.2.13/android-studio-2024.1.2.13-linux.tar.gz)
3. DJI developer account + API key from [developer.dji.com](https://developer.dji.com/)

## Install the app

```bash
git clone https://github.com/WildDrone/WildBridge.git
```

1. Open `WildBridge/WildBridgeApp/android-sdk-v5-as` in Android Studio.
2. Create `local.properties` from the template and set the Android SDK path for your machine:

   ```bash
   cd WildBridge/WildBridgeApp/android-sdk-v5-as
   cp local.properties.example local.properties
   ```

   Example for a default Linux Android Studio install:

   ```properties
   sdk.dir=/home/your-user/Android/Sdk
   ```

3. Add your DJI API key to `local.properties`:

   ```properties
   AIRCRAFT_API_KEY="Your_App_Key"
   ```

4. Build and deploy to your RC or Android phone (enable Developer Mode + USB Debugging first).

## Command-line build

```bash
cd WildBridge/WildBridgeApp/android-sdk-v5-as
./gradlew :app:assembleCurrentDebug
./gradlew :app:assembleDemoBiomassDebug
```

The debug APKs are written to:

```text
WildBridgeApp/wildbridge-app/build/outputs/apk/current/debug/WildBridge-debug.apk
WildBridgeApp/wildbridge-app/build/outputs/apk/demoBiomass/debug/WildBridge-debug.apk
```

To build/install a selected variant when an Android device is connected over ADB:

```bash
cd WildBridgeApp/android-sdk-v5-as
./auto_install_on_connect.sh current --build
./auto_install_on_connect.sh demo_biomass --build
```

To only check which APK will be used:

```bash
./auto_install_on_connect.sh current --check
./auto_install_on_connect.sh demo_biomass --check
```

## Start the server

1. Launch the WildBridge app on the RC — servers start automatically on the default layout.
2. Note the Device IP shown in the app, call `/config`, or use auto-discovery.
3. Press **Enable Virtual Stick** (via `/send/enableVirtualStick` or the app UI) before sending navigation commands.

## Ground station dependencies

```bash
pip install -e GroundStation/Python                     # Python interface
pip install -r GroundStation/ROS/requirements.txt      # ROS 2 interface
```

## Connect and control

### Telemetry (TCP, port 8081)

```python
import socket, json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.1.100", 8081))
buffer = ""
while True:
    buffer += sock.recv(4096).decode('utf-8')
    while '\n' in buffer:
        line, buffer = buffer.split('\n', 1)
        if line.strip():
            t = json.loads(line)
            print(f"Battery: {t['batteryLevel']}%  Alt: {t['location']['altitude']:.1f}m  Sats: {t['satelliteCount']}")
```

### Commands (HTTP POST, port 8080)

```python
import requests

rc = "192.168.1.100"
requests.post(f"http://{rc}:8080/send/takeoff")
requests.post(f"http://{rc}:8080/send/gotoWaypointNoseForward", data="49.306254,4.593728,20,90,5.0")
requests.post(f"http://{rc}:8080/send/navigateTrajectoryDJINative",
              data="10.0;49.306,4.593,20;49.307,4.594,25;49.308,4.595,20")
requests.post(f"http://{rc}:8080/send/RTH")
```

### Video (WHIP/WHEP through MediaMTX)

```bash
docker compose -f GroundStation/video_test/compose.yaml up -d --build
```

Open the dashboard at <http://localhost:8090>. When the dashboard connects to a phone telemetry stream, the app builds a WHIP publish URL such as:

```text
http://<ground-station-ip>:8889/<drone_name>/whip
```

MediaMTX exposes the matching browser playback endpoint:

```text
http://<ground-station-ip>:8889/<drone_name>/whep
```

The supported public video example is defined by [compose.yaml](https://github.com/WildDrone/WildBridge/blob/main/GroundStation/video_test/compose.yaml), [mediamtx.yml](https://github.com/WildDrone/WildBridge/blob/main/GroundStation/video_test/mediamtx.yml), and the webapp in `GroundStation/video_test/webapp`. The older direct WebSocket-signaling viewer/server path has been removed from the public app and ground-station tooling.

## Next steps

- Use the high-level Python client instead of raw sockets: [Ground Station](/groundstation/).
- Browse every command endpoint: [HTTP API](/http-api/).
- Fly from QGroundControl: [MAVLink 2](/mavlink/).
