# MAVLink work — handoff

State as of the end of this session. Everything described here is committed and pushed.

- **WildBridge**: `feat/mavlink-telemetry` → `origin` (`alejp1998/WildBridge`)
- **Swarm-Steward**: `feat/wildbridge-mavlink-transport` → `origin` (`alejp1998/dialogue-swarm`)

The design reasoning lives in [COMMAND_CORE_REFACTOR.md](COMMAND_CORE_REFACTOR.md) and in the
shared artifact at <https://claude.ai/code/artifact/c84b887a-1c2a-4caa-8c34-b97fe3b86ba9>. This
file is the operational picture: what works, what has never been exercised, and how to pick it up.

## Where it stands

WildBridge speaks MAVLink 2 as a vehicle. A stock QGroundControl connects, sees telemetry and
video, and flies it. Swarm-Steward's ROS stack runs on the same wire, selected by one environment
variable. Both can watch the same aircraft at once.

| | |
|---|---|
| Commands the ROS controller uses | **27 of 27** have a MAVLink form |
| `/send/set*` settings endpoints | **14 of 14** (numeric via `PARAM_SET`, string via `PARAM_EXT_SET`) |
| Telemetry keys carried | **33 of 47** — the rest are HTTP-only by nature (detections, WebRTC, phone location) |
| Agreement between the two wires | **0 disagreements** against a live aircraft |
| Tests | 89 JVM, 90 Python, `pre-commit --all-files` clean |

## The one thing that has never run

**The onboard mission sequencer has never flown a plan.** Upload, download, every refusal path and
the wire format are verified against a real aircraft; the loop that actually flies a plan has only
ever run in a compiler. It is the largest untested thing in the stack.

When flying it the first time: short plan, two or three legs, low altitude, hand on the sticks.
Grabbing the sticks must stop it — the manual-override latch cancels the control loop in
`DroneController`, which is transport-independent, and the sequencer polls it. Also unexercised
in flight: the arrival dwell, the `positionReached` fix, and the takeoff-altitude climb.

## Running it

```bash
# Ground station transport, in dialogue-swarm/config/deployments/wildbridge/wildbridge.env
WB_TRANSPORT=mavlink        # or http (default) or both
WB_MAVLINK_PORT=14551       # where the ROS nodes listen
WB_MAVLINK_PEER_PORT=14550  # where the aircraft listens

# The debug webapp, from the WildBridge repo
docker compose -f compose.video-test.yaml build video-grid
DRONE_FALLBACKS=mini1=<drone-ip> \
  docker compose -f compose.video-test.yaml up -d --no-deps video-grid
# → http://localhost:8090, MAVLink tab
```

`docker restart` does **not** re-read `env_file`. An environment change needs
`up -d --force-recreate`, which cost a debugging round once already.

### Ports

One UDP port each, because a datagram goes to exactly one socket. All three send to the aircraft
on 14550.

| | Listens |
|---|---|
| QGroundControl | 14550 |
| Swarm-Steward ROS nodes | 14551 |
| Debug webapp panel | 14552 |

A fleet needs one port per aircraft for the same reason.

## The tool that found almost everything

The **MAVLink tab** in the webapp reads one drone over both wires at the same moment and lists the
fields where they disagree, differing rows first. Every MAVLink defect in this project was found
this way and none from reading code, because they were all the same species: a frame that decodes
cleanly and means the wrong thing.

It found, within minutes of first being pointed at an aircraft, a status message being decoded
twelve bytes out of alignment because the drone was on an older build.

**Use it before believing anything about this stack.** `/api/drones/<name>/mavlink` returns the
same comparison as JSON if a script is easier than a browser.

## What is left

1. **Fly a mission** — see above. Nothing else comes close in importance.
2. **`intermediaryWaypointReached`** — the one telemetry key with no MAVLink form. It is published
   by the ROS node and subscribed by nothing, so it may simply be dead; check before building it.
3. **Signing in anger** — verification is implemented and checked against pymavlink's own signer,
   but no Safety Computer has ever sent a signed frame. Set `wb_mav_0_signing_key` on the aircraft
   to 64 hex characters and have the safety side sign with the same key; a signed frame then maps
   to `ControlAuthority.Source.SAFETY`, exactly as `X-Safety-Token` does over HTTP.
4. **Media transfer** — `list_media` / `download_media` are still HTTP. MAVLink FTP is the route.
5. **`MAV_CMD_USER_1/2`** — the payload residue rides on two user commands with a selector in
   `param1`. It works, but `wildbridge.xml` exists now and named commands there would be honest.
6. **Superseded acks** — a re-issued goto now acks `MAV_RESULT_CANCELLED` rather than `FAILED`.
   Confirm Swarm-Steward's action layer treats that as bookkeeping, not an error.

## Things that will bite

- **`WILDBRIDGE_STATUS` is decoded by hand** in `transport.py`, not by a generated dialect. Its
  layout and CRC_EXTRA (**216**) come from `GroundStation/mavlink/wildbridge.xml`. If you add a
  field there, regenerate with mavgen and update `WILDBRIDGE_STATUS_STRUCT`, `_SIZE` and
  `_CRC_EXTRA` together. A test regenerates from the XML and fails if they drift.
- **Both ends must be on the same build** once that message changes. The ground station now
  verifies the checksum and refuses a mismatched frame rather than misreading it, so the symptom
  is fields quietly missing rather than garbage — check the aircraft's APK first.
- **MAVLink packs fields largest-first**, not in XML order. `failure_flags` (u32) precedes `flags`
  (u16) despite being declared after it. Getting that wrong produces a valid checksum over
  meaningless bytes; it has happened twice.
- **CRC_EXTRA above 127 must be accumulated as a byte**, not through a `str`. In UTF-8 it becomes
  two bytes and every genuine frame fails. The test written alongside that check passed anyway,
  because the fixture repeated the mistake.
- **DJI's unset marker is 6553.5** (65535/10) on gimbal axes, and `-1` on several settings. Both
  are real-looking numbers. Treat implausible values as unknown rather than publishing them.

## Gimbal joint angles, settled

Calibrated by logging both wires while the aircraft was tilted by hand — 91 samples, script at
`gimbal_capture.py` in the session scratchpad, method worth repeating for any similar question.

The derivation (`q_body⁻¹ ⊗ q_world`) has the right sign and a slope of **+1.03**, so
`GIMBAL_JOINT_SIGN` stays `+1`. What it cannot recover is a **~1.5° mounting offset** baked into
DJI's joint angles. The aircraft therefore reports them directly in `WILDBRIDGE_STATUS`; the
composition survives only as a fallback. The two wires now agree to a tenth of a degree.
