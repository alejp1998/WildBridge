<div align="center">
    <img src="docs/images/WildBridge_icon.png" alt="WildBridge App Icon" width="180" height="180">

**Ground Station Interface for Lightweight Multi-Drone Control and Telemetry on DJI Platforms**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![DJI MSDK V5](https://img.shields.io/badge/DJI%20MSDK-V5.18.0-blue.svg)](https://developer.dji.com/doc/mobile-sdk-tutorial/en/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-brightgreen.svg)](https://docs.ros.org/en/humble/)
[![Part of WildDrone](https://img.shields.io/badge/Part%20of-WildDrone-orange.svg)](https://wilddrone.eu)

*Part of the [WildDrone Project](https://wilddrone.eu) — European Union Horizon Europe Research Programme*

</div>

---

## What is WildBridge?

WildBridge is an **open-source Android application** (Kotlin, DJI Mobile SDK V5) that runs on the DJI Remote Controller and turns it into a networked drone server. Telemetry, commands, and live video leave the aircraft over standard, open protocols — **HTTP, TCP, MAVLink 2, and WebRTC** — so any ground station, in any language, can fly a DJI drone without touching DJI's proprietary SDK.

![WildBridge System Architecture](docs/images/WildBridgeDiagram.png)

## Why WildBridge?

- ✈️ **Full flight control** — waypoint missions (two PID modes), DJI-native missions, virtual stick, yaw, altitude, RTH
- 🌐 **Protocols, not SDKs** — REST commands, streaming JSON telemetry, a complete MAVLink 2 vehicle, WHIP/WHEP video
- 🛡️ **Two-computer safety** — a Safety Computer can seize command authority at any time, and only it can hand control back
- 🤖 **ROS 2 ready** — 45+ topics per drone, dynamic namespaces, zero-config auto-discovery
- 📡 **QGroundControl & MAVSDK compatible** — each drone appears as a standard MAVLink 2 vehicle, no plugin needed
- 🔥 **Enterprise sensors** — thermal capture and temperature, laser rangefinder, payload drop
- 🧭 **Mission-proven** — zebra-herd monitoring, wildfire detection (XPRIZE Wildfire semi-finalist), wind-field profiling

## Supported hardware

DJI Mini 3 / Mini 4 Pro · Mavic 3 Enterprise · Matrice 30 / 300 RTK / 350 RTK / 4 Thermal — flown from the DJI RC Pro, RC Plus, or RC-N3.
[Full list](https://developer.dji.com/doc/mobile-sdk-tutorial/en/)

## In the field

WildBridge has been used in the following research applications (Rolland et al., RiTA 2025):

| Study | UAVs | Features | Video |
|-------|------|----------|-------|
| Drone Swarm for Wildlife Monitoring | 2× Mini 3, 1× M3E | Telemetry, video, waypoints | [▶ Watch](https://www.youtube.com/watch?v=PzHnbgxLaSU) |
| Drone Swarm for Wildfire Detection | 1× M3E, 1× M4T, 2× M300 | Thermal detection, coordinated take-off, payload drop; XPRIZE Wildfire semi-finalist | [▶ Watch](https://www.youtube.com/watch?v=F73VcUoOzo8) |
| Atmospheric Wind Field Profiling | 3× Mini 3 | Vertical wind profiles validated against LiDAR | [▶ Watch](https://www.youtube.com/watch?v=KZ40L-y1xt8) |
| Custom PID Position Controller | — | On-device PID controller | [▶ Watch](https://www.youtube.com/watch?v=j52ovMPVt_I) |

## Documentation

The full manual — quick start, HTTP API, telemetry, MAVLink 2, ROS 2, and field-test procedures — lives in the docs site:

### [wilddrone.github.io/WildBridge](https://wilddrone.github.io/WildBridge/)

Run it locally:

```bash
npm install
npm run dev        # http://localhost:4321/WildBridge/
```

## Quick start

```bash
git clone https://github.com/WildDrone/WildBridge.git && cd WildBridge
pip install -e GroundStation/Python            # Python ground-station client
```

For the Android app, open `WildBridgeApp/android-sdk-v5-as` in Android Studio, copy `local.properties.example` to `local.properties` (set `sdk.dir` and `AIRCRAFT_API_KEY`), build the `current` variant, and install it on the RC. The servers start automatically on launch.

**Next:** follow the [Getting Started guide](https://wilddrone.github.io/WildBridge/getting-started/) to connect your first ground station.

---

## Funding

This work is part of the **WildDrone** project, funded by the European Union's Horizon Europe Research Programme under the Marie Skłodowska-Curie Grant Agreement No. 101071224, with additional funding from the EPSRC grant *Autonomous Drones for Nature Conservation Missions* (EP/X029077/1) and the Independent Research Fund Denmark (10.46540/4264-00105B).

## Citation

Edouard G.A. Rolland, Kilian Meier, Murat Bronz, Aditya M. Shrikhande, Tom Richardson, Ulrik P.S. Lundquist, Anders L. Christensen — *WildBridge: Ground Station Interface for Lightweight Multi-Drone Control and Telemetry on DJI Platforms*, Proceedings of the 13th International Conference on Robot Intelligence Technology and Applications (RiTA 2025), Springer. [Publication page](https://portal.findresearcher.sdu.dk/en/publications/wildbridge-ground-station-interface-for-lightweight-multi-drone-c)

```bibtex
@inproceedings{Rolland2025WildBridge,
  author    = {Edouard G.A. Rolland and Kilian Meier and Murat Bronz and
               Aditya M. Shrikhande and Tom Richardson and
               Ulrik P.S. Lundquist and Anders L. Christensen},
  title     = {{WildBridge}: Ground Station Interface for Lightweight
               Multi-Drone Control and Telemetry on {DJI} Platforms},
  booktitle = {Proceedings of the 13th International Conference on
               Robot Intelligence Technology and Applications (RiTA 2025)},
  year      = {2025},
  month     = {December},
  publisher = {Springer},
  address   = {London, United Kingdom},
  note      = {In press},
  url       = {https://portal.findresearcher.sdu.dk/en/publications/wildbridge-ground-station-interface-for-lightweight-multi-drone-c},
}
```

## Contributors

Major contributors to WildBridge's development and field testing who are not listed as authors on the paper above:

- [Alejandro Jarabo-Peñas](https://alejp.me)
- [Juan Bravo-Arrabal](https://www.linkedin.com/in/juan-bravo-arrabal)

## License

MIT License — see [LICENSE](LICENSE) for details.

Bug reports and feature requests: [GitHub Issues](https://github.com/WildDrone/WildBridge/issues).
For collaboration enquiries, contact the WildDrone consortium at [wilddrone.eu](https://wilddrone.eu).
