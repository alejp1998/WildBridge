---
title: Logs & Troubleshooting
description: Flight logging locations, common connection issues, and the repository layout.
breadcrumb: Operations
---

## Flight logging

WildBridge logs flight data in JSONL format.

Storage locations are checked in order:

1. Removable microSD card: `WildBridge/FlightLogs/YYYY-MM-DD/HH-mm-ss_<drone>.jsonl`
2. Documents folder: `Documents/WildBridge/FlightLogs/YYYY-MM-DD/`
3. App-external fallback: `Android/data/<pkg>/files/FlightLogs/YYYY-MM-DD/`

DJI SDK TXT flight records are copied to `WildBridge/DJI_FlightRecords/` on app launch and after landing so they survive app reinstalls.

## Troubleshooting

**Connection refused:**

- Verify the WildBridge app is running on the RC (servers start on launch).
- Check the RC is on the same LAN as the GS.
- Test with `curl http://{RC_IP}:8080/config` and `nc {RC_IP} 8081`.

**Drone does not respond to navigation commands:**

- Press **Enable Virtual Stick** in the app or call `/send/enableVirtualStick`.
- Check `isManualOverrideActive` in telemetry; call `/send/deactivateManualOverride` if needed.

**Video not connecting:**

- Start the video-test stack with `docker compose -f GroundStation/video_test/compose.yaml up -d --build`.
- Open <http://localhost:8090> and verify the phone telemetry connection is active.
- Check MediaMTX paths with `curl http://localhost:9997/v3/paths/list`.
- Prefer clean 5 GHz Wi-Fi channels for multiple simultaneous video publishers.

**Android build:**

```bash
cd WildBridgeApp/android-sdk-v5-as
./gradlew :app:compileCurrentDebugKotlin
./gradlew :app:assembleCurrentDebug
./gradlew :app:assembleDemoBiomassDebug
```

## Project structure

```text
WildBridge/
├── WildBridgeApp/
│   ├── android-sdk-v5-as/               # Main Android project (open this in Android Studio)
│   │   └── local.properties             # Place AIRCRAFT_API_KEY here
│   ├── wildbridge-app/                 # WildBridge Android app (com.wildbridge.rc)
│   │   └── src/main/
│   │       └── java/com/wildbridge/rc/
│   │           ├── WildBridgeDefaultLayoutActivity.kt
│   │           ├── controller/          # DroneController, PID, autonomy helpers
│   │           ├── logger/              # Flight and DJI record logging
│   │           ├── server/              # HTTP, telemetry, and discovery services
│   │           └── webrtc/              # WHIP/WebRTC video publishing
│   └── android-sdk-v5-uxsdk/            # DJI UXSDK UI components
└── GroundStation/
    ├── Python/
    │   ├── wildbridge_groundstation/    # Canonical DJI client (HTTP + TCP telemetry)
    │   ├── djiInterfaceSafety.py        # Legacy import shim for wildbridge_groundstation.safety
    │   └── test_scripts/                # Authority and capture/download test scripts
    ├── Dockerfile                       # ros:humble + CycloneDDS container
    ├── entrypoint.sh                    # Container entry point
    ├── run_docker.sh                    # Docker run helper
    ├── video_test/                      # MediaMTX + multi-drone video dashboard
    │   └── compose.yaml                 # MediaMTX + dashboard compose stack
    └── ROS/
        ├── wildbridge_controller/       # ROS 2 control + telemetry node
        ├── wildbridge_videofeed/        # Video feed node
        └── wildbridge_bringup/          # Launch files and config
```
