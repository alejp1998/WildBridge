# MAVLink Command-Core Refactor — Handoff

Branch: `feat/mavlink-telemetry` · Started from `1fa4641` · Updated 2026-08-26

This file is the working handoff for the "command core" phase of the WildBridge-on-MAVLink
plan. It exists so another agent (or a future session) can pick up exactly where the work left
off without re-deriving the context.

The plan document itself is an exported Claude artifact: `~/Downloads/WildBridge on MAVLink.html`
(the rendered content is in `~/Downloads/WildBridge on MAVLink_files/saved_resource.html`).
The relevant section is **§15 "The revised plan", Phase 2 — Modes and command core**.

---

## 1. The problem being solved

WildBridge has two surfaces that accept commands, and they are implemented twice:

- **HTTP** — `WildBridgeHttpServer.kt` (`WildBridgeHttpCommandHandler.postRoutes`, ~44 routes).
  Each route returns a plain English string, so a ground station learns a command was refused by
  matching the literal text `"REJECTED:"`.
- **MAVLink** — `mavlink/MavlinkCommandSink` (typed) → `mavlink/MavlinkTelemetryEndpoint`.
  Returns `MavlinkCommandOutcome`, which maps straight onto `MAV_RESULT`.

Two parallel implementations of the same commands is precisely what killed the old
`mavlink_proxy.py` / `wildbridge_mavros` (deleted in commit `49337c4`): every endpoint added to
the app made the proxy quietly more wrong. The refactor collapses the two surfaces onto one typed
command layer so they cannot drift.

**Decision made:** do the command-core refactor (drone-free, load-bearing) before the motion
groundwork. Motion (takeoff/land/RTH/reposition/yaw/altitude) stays gated behind Phase 3's
`ControlAuthority` + 2 Hz offboard failsafe.

## 2. Design

A shared result type, added to `mavlink/MavlinkCommandSink.kt`:

```kotlin
internal data class CommandResult(
    val outcome: MavlinkCommandOutcome,
    val detail: String? = null
) {
    val mavResult: Int get() = outcome.mavResult
}
```

- `outcome` is the MAV_RESULT-shaped result both surfaces share (`MavlinkCommandOutcome` is
  unchanged: `ACCEPTED / FAILED / DENIED / UNSUPPORTED`).
- `detail` carries the **byte-for-byte HTTP prose**, produced once by the command layer.
- **HTTP becomes a pure renderer** (reads `detail`).
- **MAVLink reads only `outcome.mavResult`** and ignores `detail`.
- Contract: HTTP output must not change ("nothing user-visible moves").

## 3. What is already done (uncommitted, compiles green)

All changes compile cleanly (`:app:compileCurrentDebugKotlin` → BUILD SUCCESSFUL).

**Stage 1 — foundation (zero behaviour change):**

- `mavlink/MavlinkCommandSink.kt` — the 5 sink methods now return `CommandResult`; added the
  `CommandResult` data class; `MavlinkCommandOutcome` enum kept as-is.
- `mavlink/MavlinkTelemetryEndpoint.kt` — `executeCommand` now builds `CommandResult` and returns
  `result.mavResult`.
- `WildBridgeDefaultLayoutActivity.kt` — the 5 `MavlinkCommandSink` implementations and the
  `awaitAction` helper return `CommandResult`; added `import ...mavlink.CommandResult`.
- New test `src/test/java/com/wildbridge/rc/mavlink/CommandResultTest.kt` (JUnit 4).

**Stage 2 — first slice: HTTP payload commands now go through the shared sink:**

- `MavlinkCommandSink` is injected into `WildBridgeHttpCommandHandler`/`SimpleHttpServer` via
  constructor (from the activity's `mavlinkCommandSink` in `startServers()`), **not** through the
  `WildBridgeCommandHost` interface — an `internal` type can't be exposed by a public class's
  override, so the interface route is a dead end. Keep it this way.
- `/send/camera/zoom`, `/send/camera/startRecording`, `/send/camera/stopRecording` now call
  `commandSink.*` and render from the outcome. Success text is unchanged (byte-for-byte);
  failure paths are new and honest.
- Behaviour changes introduced on purpose: zoom now rejects `<= 0` with `"Invalid zoom value"`;
  recording now blocks on `awaitAction` and can return `"FAILED: camera ..."`.

## 4. What remains (Stage 2 onward)

1. **Gimbal — DONE (uncommitted).** The four HTTP gimbal routes (`/send/gimbal/pitch`, `/yaw`,
   `/rel_pitch`, `/rel_yaw`) and the MAVLink `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` now all go
   through `MavlinkCommandSink.setGimbal(rotation: GimbalRotation)`. Added a new SDK-free
   `mavlink/GimbalRotation.kt` (mode + roll/pitch/yaw + the three per-axis `*Ignored` flags,
   mirroring DJI's `GimbalAngleRotation` minus the constant duration/joint/timeout fields). The
   `*Ignored` flags per route were preserved byte-for-byte. Also fixed the MAVLink path, which had
   `pitchIgnored = true` (so `DO_GIMBAL_MANAGER_PITCHYAW` never moved pitch): now `false`, since
   the aircraft no-ops an axis the gimbal lacks.
2. Unify the remaining payload route, capture: HTTP has `/send/captureTemperature` and
   `/send/captureThermalImage` (JSON, in `handleJsonEndpoint`) plus `Payload.capturePhoto` — the
   MAVLink side is `captureImage()`. Decide the shared shape before wiring.
3. Then the settings/misc routes, then the motion slice + `ControlAuthority` gate + QGC actions
   JSON (Phase 3). **Do not add motion commands to `MavlinkCommandSink` until the Phase 3 gate
   exists** — the sink's doc comment promises "nothing here can move the aircraft".

## 5. Constraints / subtleties

- Byte-for-byte HTTP prose preservation is the contract of this phase.
- The 4 gimbal routes map to more than one sink operation — the command layer must carry the full
  request shape (mode, axes), not just an outcome enum.
- `MavlinkCommandSink` must stay free of DJI SDK imports (the endpoint package is SDK-free by
  design); implementations live in `WildBridgeDefaultLayoutActivity`.

## 6. Environment facts (Android build)

- Gradle variant tasks are `compileCurrentDebugKotlin` and `testCurrentDebugUnitTest`
  (variants: `currentDebug`, `demoBiomassDebug`). `compileDebugKotlin` is **ambiguous** and
  fails; `AGENTS.md`'s examples are stale on this point.
- **Unit tests cannot run here**: the Gradle Test Executor fails to start for *all* tests,
  including the pre-existing `WildBridgeHttpCommandParserTest` — a pre-existing environment issue,
  not caused by this refactor. The Kotlin daemon also fails and falls back to non-daemon compile
  (which still succeeds).
- `window_dump.xml` at the repo root is an untracked QGC/uiautomator dump — junk, do not commit.

## 7. Prior context (already resolved in earlier sessions)

- Python quality gates are all green (ruff lint/format, radon, mypy, bandit) and `pytest` passes.
- Telemetry rate bug fixed (scheduler overshoot): ATTITUDE ~9.81 Hz vs 10 Hz target.
- `AGENTS.md` corrected (it pointed at a deleted `GroundStation/Python/requirements.txt`).
- The `mediamtx` healthcheck complaint is **not** WildBridge's — it belongs to the
  `dialogue-swarm` repo. WildBridge's own compose builds a custom image with busybox and is fine.
- No caller ever used the broken `host.` recording route form — the rename is safe.

## 8. Multi-vehicle (QGC) — unique system id

QGC merges two vehicles that share a MAVLink system id (its "Multi-Vehicle Gotchas" note). Before
this change every WildBridge device shipped with `wb_mav_0_sysid` defaulting to `1`, so two drones
on one subnet would appear as one vehicle.

Implemented (uncommitted, same branch):

- `mavlink/MavlinkSystemId.kt` — pure helper: `fromKey()` maps a key deterministically into 1..254
  (`String.hashCode()` is JLS-stable), `resolve(configured, key)` treats `0` as auto and clamps
  explicit ids. 0 and 255 are reserved and never produced.
- `mavlink/MavlinkEndpointConfig.kt` — `DEFAULT_SYSTEM_ID` is now `MavlinkSystemId.AUTO` (0).
- `WildBridgeDefaultLayoutActivity.readMavlinkConfig()` — resolves the id via `MavlinkSystemId`,
  keyed off the drone name once renamed (`sysIdKey()`), else the aircraft serial number. The name
  defaults to `"drone_1"` on every device, so name-only would collapse un-renamed drones.
- Test: `src/test/java/com/wildbridge/rc/mavlink/MavlinkSystemIdTest.kt`.

QGC is running on the GCS (`192.168.50.127`, UDP 14550). Test phones:
`192.168.50.224` (name `mini3` → sysid 129) and `192.168.50.136` (name `mini7` → sysid 133).
Both have the new APK and `wb_mav_0_enabled=true`, host `.127`. Prefs are `WildBridgePrefs.xml`
under `shared_prefs/`, edited by hand via `adb run-as` (force-stop first).

Device test status at handoff: `.224`'s MAVLink endpoint is confirmed live (UDP 14550 bound).
`.136` shows no listening sockets at all (HTTP 8080/8081 also absent) — the app process exits
cleanly on launch, so this is a pre-existing device-side startup issue, NOT the sysid change
(which runs after the HTTP/telemetry blocks in `startServers()`). Needs a manual launch on the
`.136` screen / DJI registration check.

## 9. Multi-vehicle video switching — fixed

QGC would not switch the video when changing the active drone. Root cause: `VIDEO_STREAM_INFORMATION`
was sent **only on request** and returned `MAV_RESULT_DENIED` while the WHIP publish was not yet up
(the publish starts asynchronously when a TCP telemetry client attaches to port 8081). A drone that
was asked before its stream was live got cached as "no stream" and never re-advertised.

Fix (uncommitted): stream `VIDEO_STREAM_INFORMATION` from the camera component at 1 Hz in
`MavlinkTelemetryEndpoint.buildStreams()`, guarded by `sendIf = { videoStreamProvider()?.uri?.isNotBlank() == true }`,
mirroring the existing `CAMERA_CAPTURE_STATUS` "streamed as well as served on request" pattern.
Deployed to both phones. Verify by switching active vehicle in QGC — MediaMTX (`:9997/v3/paths/list`)
should then show an `rtspSession` reader on the newly selected drone's path.

Note: vehicle *names* still can't be shown in QGC's multi-vehicle list/map (hard-coded to
"Vehicle <sysid>"); the component-metadata `name` only surfaces in the Vehicle view. Not implemented.

## 10. Flight-motion commands (Phase 3) — implemented

Discrete flight-motion over MAVLink, kept in a **separate sink** from the payload commands so the
payload sink's "nothing here can move the aircraft" contract stays true.

- `mavlink/MavlinkMotionSink.kt` — SDK-free interface: `takeoff()`, `land()`, `returnToHome()`,
  `reposition(latDeg, lonDeg, altM, yawDeg, groundSpeedMps)`, `setYaw(yawDeg)` — all return `CommandResult`.
- `mavlink/MavlinkProtocol.kt` — added `CMD_NAV_RETURN_TO_LAUNCH=20`, `CMD_NAV_LAND=21`,
  `CMD_NAV_TAKEOFF=22`, `CMD_CONDITION_YAW=115`, `CMD_DO_REPOSITION=192`, `CMD_COMPONENT_ARM_DISARM=400`.
- `mavlink/MavlinkInbound.kt` — `MavlinkCommand` gained `param5/param6/param7: Float`; parser
  normalizes `COMMAND_INT` (`int32 / 1e7`) vs `COMMAND_LONG` (raw float) for param5/6, raw float for 7.
- `mavlink/MavlinkTelemetryEndpoint.kt` — constructor takes `motionSink: MavlinkMotionSink? = null`;
  `executeCommand` dispatches TAKEOFF/LAND/RTL/DO_REPOSITION/CONDITION_YAW. `COMPONENT_ARM_DISARM`
  → `UNSUPPORTED` (DJI has no arming concept).
- `WildBridgeDefaultLayoutActivity.kt` — `mavlinkMotionSink` object implements the sink, passed as
  the 6th endpoint arg. `DroneController.startTakeOff/startLanding/startReturnToHome/` +
  `flyToWaypointHoldHeading(...)` + `gotoYaw(...)`.

**3-way gate (in order), inside `mavlinkMotionSink`:**

1. `wb_mav_0_allow_flight` pref (`MavlinkEndpointConfig.PREF_ALLOW_FLIGHT`) — ships **false**,
   so nothing moves until deliberately enabled per device.
2. `ControlAuthority.authorizeControlCommand(Source.PILOT)` — MAVLink speaks as Pilot, so it is
   refused once the Safety Computer has seized control over HTTP.
3. `DroneController.shouldRejectAutonomousCommand("reposition"/"yaw")` — refuses the closed-loop
   commands while the physical RC pilot has manually overridden.

TAKEOFF/LAND/RTL use only gate layers 1+2 (they are the pilot's own abort path). Reposition/yaw
additionally pass layer 3.

**QGC buttons:** `qgc/wildbridge-actions.json` — Takeoff (22, param7=5 m), Land (21), RTL (20).
Load via Application Settings → MAVLink Actions → Fly View Actions. Reposition/yaw are dynamic
(lat/lon/heading), so they're sent from a GCS script or the MAVLink console, not static buttons.

**Field state (2026-08-26):** built `sample-currentDebug.apk` and flashed to the USB phone
(`df3a6d5`, name `mini1`); set `wb_mav_0_allow_flight=true` on it and relaunched. UDP 14550 bound
(endpoint live). ⚠ That phone's `wlan0` is now `10.82.87.30` while `wb_mav_0_host` is still the
field `192.168.50.127` — update the host (or rejoin the field WiFi) so directed traffic reaches
QGC; broadcast heartbeats still work on the local subnet.

**Not implemented (deferred):** the 2 Hz offboard failsafe — that's for continuous offboard
control, not the discrete commands above.

## 11. QGC built-in Takeoff/Land/RTL buttons — fixed (uncommitted)

**Symptom:** QGC showed the vehicle, video and telemetry and said "Ready", but the Fly View
Takeoff/Land/RTL buttons stayed grey.

**Root cause (verified in QGC source):** QGC enables those buttons only when the vehicle's
firmware plugin declares capabilities. The plugin is chosen by `HEARTBEAT.autopilot`; WildBridge
reported `MAV_AUTOPILOT_INVALID` (8), which selects the Generic plugin, whose `isCapable()`
returns false for everything. The buttons are gated on `VehicleSupports` →
`firmwarePlugin()->isCapable(...)`, so they could never enable regardless of "Ready".

**Fix — claim `MAV_AUTOPILOT_PX4` (12), which was always the documented fallback:**

- `MavlinkMessages.heartbeat()` now reports `AUTOPILOT_PX4`. The camera component's heartbeat
  stays `AUTOPILOT_INVALID` (correct for non-autopilot components).
- `MavlinkFlightMode` custom-mode values are now PX4's packed numbers
  (`(main << 16) | (sub << 24)`, mirrored from QGC's `px4_custom_mode.h`), so the mode indicator
  renders matching names: Position/Altitude/Offboard/Mission/Takeoff/Land/Return/Orbit/Manual.
  PX4 was chosen over ArduPilot because PX4 *has* Takeoff/Land/Return as named modes, and its
  takeoff flow is command-driven (ArduPilot's validates a mode change and arming through
  heartbeats, which a DJI adapter cannot honestly emulate).
- `EXTENDED_SYS_STATE` (MAVLink 2 message id 245) streamed at 1 Hz: `landed_state` is QGC's only
  source of "flying",
  and Land/RTL buttons require `armed && flying`. Derived from `motorsRunning` (DJI `isFlying`),
  with LANDING/TAKEOFF states during the matching DJI modes.
- `SET_MODE` (11) parsed inbound (`MavlinkSetMode` + `parseSetMode`) — QGC's PX4 plugin asks for
  Land/RTL with this message, not `DO_SET_MODE`. Both routes land in `modeResult()`, which maps
  the requested mode back via `MavlinkFlightMode.fromPx4Mode()` and acts: TAKEOFF → takeoff,
  LAND → land, SAFE_RECOVERY → RTH. Other modes → UNSUPPORTED (DJI has no remote mode switching).
  Acting on the request also makes DJI report the matching mode, which is what QGC's
  validation loop watches for.
- `MAV_CMD_COMPONENT_ARM_DISARM` now → `arm()`/`disarm()` on `MavlinkMotionSink`: gated no-op
  ACKs (DJI arms when the takeoff actually starts). Required because QGC's PX4 plugin arms right
  after `NAV_TAKEOFF` is accepted and aborts on refusal.
- `MAV_CMD_DO_SET_MODE` (176) also routed to `modeResult()` (APM-plugin compatibility; PX4 uses
  SET_MODE). Packed mode numbers fit a float32 exactly (low 16 bits are zero).
- CRC extras added: `SET_MODE to 89`, `EXTENDED_SYS_STATE to 130` (the `to 22` was the bug
  section 12 documents).

**Deployed:** rebuilt and flashed to the USB phone (`df3a6d5`, mini1); prefs survived
(`wb_mav_0_allow_flight=true`), UDP 14550 bound. QGC must be re-connected to the vehicle (the
autopilot field changed, so the existing vehicle object is stale).

## 12. Field follow-ups — flying-state CRC bug + PX4 dialogs (uncommitted)

**Bug found in the field:** takeoff worked, but after takeoff completed the Land/RTL buttons never
appeared. Root cause: `MavlinkCrc.CRC_EXTRA` listed `EXTENDED_SYS_STATE` as **22**; the correct
value is **130** (verified against the official `mavlink/c_library_v2` generated header). QGC
drops every frame whose checksum does not match, so it never received a single
`EXTENDED_SYS_STATE` — `Vehicle.flying` stayed false forever, which keeps `showTakeoff` true and
hides `showLand` (`showLand && !showTakeoff`). Fixed to 130.

The same investigation verified `SET_MODE` (89) and the rest of the emitted-message CRC table
against pymavlink/c_library_v2 — everything else was already correct.

**The three PX4 dialogs after claiming the autopilot, and their fixes:**

1. *"Vehicle is not running latest stable firmware! Running -1.-1.-1"* — QGC's initial-connect
   state machine parses `AUTOPILOT_VERSION.flight_sw_version` and, when zero, leaves the version
   unset; the stable check then compares -1 against the latest GitHub release. Fixed by reporting
   `(1 << 24) | (15 << 16) | 0xFF` — PX4 1.15.0 official, `0xFF` = OFFICIAL type byte.
2. *"Parameters are missing from firmware... 1:SYS_AUTOSTART"* — QGC's PX4 airframe component
   requires this one parameter. Fixed by publishing `"SYS_AUTOSTART" to 4001f` (PX4's "Generic
   Quadcopter" airframe) from `mavlinkParameters()`.
3. *"QGroundControl supports PX4 Pro firmware Version 1.4.1 and above"* — same zero-version cause
   as 1; fixed by the same version report.

`flight_custom_version` now reads `WBbridge` (8-byte git-hash field) instead of zeros.

**Land vs RTL mapping (asked in the field):** QGC's Land button = DJI auto-land at the current
position (`DroneController.startLanding`); QGC's RTL button = DJI return-to-home
(`startReturnToHome`). They are separate buttons and separate DJI commands — Land is not RTH.

## 13. Field follow-up 2 — remaining dialogs + flying-state diagnostics (uncommitted)

After the CRC fix, two dialogs remained on connect (the firmware-version ones are gone):
*"Parameters are missing... 1:COM_RC_IN_MODE, 1:RC_MAP_ROLL"* and *"Configuration tasks remain
before this vehicle is ready to fly"*. Cause: QGC's PX4 Radio setup task reads `COM_RC_IN_MODE`
and the `RC_MAP_*` pins and reports them missing; with the radio task unconfigured the setup
summary says the vehicle is not ready. Fix: publish `COM_RC_IN_MODE=1` (RC from a joystick, so the
radio wizard is not required) and `RC_MAP_ROLL/PITCH/YAW/THROTTLE=0` (no MAVLink RC channels —
the DJI remote is not exposed over MAVLink) from `mavlinkParameters()`.

Also added a permanent field diagnostic to `MavlinkTelemetryEndpoint.streamLoop()`: it logs every
`motors/armed` and `landed_state`/mode transition ("State change: ..."), since QGC's Flying
indicator and Land/RTL enablement derive entirely from those two heartbeat/ESS fields. Read it
with `adb logcat -s MavlinkEndpoint`.

**Open question from the field:** after takeoff, QGC's status stayed "Armed" and Land was not
pressable — i.e., QGC still believed the vehicle was not flying. The code, the CRC table and the
built APK (verified in the dex: `125 → 130`) are all correct, so the suspect is a stale vehicle
object in QGC (it must be **fully restarted**, not just reconnected, after the autopilot claim
changed) or a phone that had not yet received the CRC-fixed build at test time. The state-change
logs above will settle it on the next flight.

> **Resolved (2026-08-26):** not a stale vehicle — the message id was wrong (125 vs 245).
> See section 14.

## 14. Field verification of the flying state (2026-08-26) — wire is correct

The remaining "no Land button" symptom was chased to ground with live captures on the field WiFi
(this machine is `192.168.50.127`, the phones' configured MAVLink target, and QGC 5.1.3 runs
here). Findings:

- The phone's `MavlinkEndpoint` log shows the state machine works: takeoff → `Command 22 ACCEPTED`,
  `Command 400 ACCEPTED` (arm), then `State change: motors/armed=true`, `landed_state=3
  mode=TAKEOFF`, then `landed_state=2 mode=POSITION_HOLD` (in air), and back to 1 on landing.
- A live UDP capture of the actual frames showed `EXTENDED_SYS_STATE` (msg 125) sent at 1 Hz from
  comp id 1 with payload `03 02` (vtol=MC, landed=IN_AIR) and a checksum that matches
  `crc_extra=130` exactly (computed by hand with the X.25 algorithm). The frame is valid standard
  MAVLink.
- pymavlink 2.4.49 *appeared* to reject these frames, but that is a pymavlink bug: its shared
  `mavlink_map` has message id 125 **overwritten by POWER_STATUS** (crc 203), so its parser uses
  the wrong message class. After repairing the table (`mavlink_map[125] =
  MAVLink_extended_sys_state_message`) the frames parse cleanly. QGC uses the C library, which
  has no such collision and declares `MAVLINK_MSG_ID_EXTENDED_SYS_STATE_CRC 130`.
- **Root cause (found and fixed the same day): the message id, not the CRC.**
  `EXTENDED_SYS_STATE` is a MAVLink 2-only message and lives at **245** in that dialect
  (`message_definitions/v1.0/common.xml`); 125 is `POWER_STATUS` in both dialects. QGC 5.1.3 is
  MAVLink 2 only and dispatches its flying-state handler for msgid 245, so it silently ignored
  every phone ESS frame — the phone's bytes were valid MAVLink, just the wrong message id.
  Proof: pymavlink's v20 dialect packs ESS at 245, and replaying those frames into QGC flipped
  `_vehicleFlying` exactly as expected, while replaying the phone's byte-for-byte 125 frames
  never did.
- **Fix (this commit):** `MavlinkProtocol.kt`: `EXTENDED_SYS_STATE = 125` → `245`. `crc_extra`
  (130) is a property of the message definition, not of the id, so `MavlinkCrc.kt` is unchanged.
  All other emitted message ids were audited against the official `common.xml` and are correct.
- **Verified:** after redeploy, the phone emits `fd 02 00 00 <seq> 7f 01 f5 ...` (msgid 245,
  payload `03 01` grounded / `03 02` in air, crc 130 valid) and nothing at 125. User
  field-verified: QGC now tracks the flying state from the phone's ESS and **Land/RTL appear
  after takeoff**.

Also published PX4 calibration params to end the whack-a-mole of missing-param dialogs:
`CAL_GYRO0_ID=131074`, `CAL_ACC0_ID=131330`, `CAL_MAG0_ID=131586` (any non-zero satisfies QGC's
Sensors setup task; DJI calibrates the IMU in the factory).



---

## Mission protocol (this session)

The mission protocol is the first microservice where WildBridge is not translating an existing
HTTP endpoint but taking over a job the ground station used to do. It is worth writing down why.

### One protocol, two executors

WildBridge already has two ways to fly a path: DJI's native wayline engine, and its own PID
controller driven waypoint-by-waypoint. The obvious reading of "we have two ways" is that we need
two MAVLink surfaces. We do not — **the choice of executor is a property of the vehicle, not of
the plan**. A ground station uploads one plan; `WB_MISSION_EXEC` decides who flies it:

| `WB_MISSION_EXEC` | Executor | Notes |
|---|---|---|
| `onboard` (default) | WildBridge's PID controller, sequenced on the aircraft | Per-item heading, per-leg speed |
| `dji_native` | DJI's wayline engine | Needs ≥ 2 waypoints; per-item heading is lost |

The one thing the plan format genuinely has to express — nose-forward versus hold-heading — is
already expressed by MAVLink itself. `NAV_WAYPOINT.param4` is the yaw at the waypoint, and **NaN
means "use the vehicle's own heading mode"**. That is exactly the distinction between WildBridge's
two waypoint controllers, so it is read per item and one plan may mix them. This is why no custom
mission item was needed.

The same convention now applies to `DO_REPOSITION` (a single goto), which previously always held
heading. A goto should not change character depending on whether it arrived as a reposition or as
a one-item plan.

### Sequencing moved onto the aircraft

Previously the ground station chained waypoints: send one, watch the reach latch, send the next.
That is why the seq-tracked reach flags exist. MAVLink expects the *vehicle* to own that state,
because `MISSION_CURRENT` and `MISSION_ITEM_REACHED` are reported by the aircraft. The onboard
sequencer in `WildBridgeDefaultLayoutActivity` does that: it walks the stored items, issues each
leg through the controller its `param4` asks for, waits on the reach latch **comparing the seq**
(a bare boolean would read a stale latch from the previous leg as this one's arrival), and reports
progress. It aborts rather than skipping on if a leg times out or the pilot takes the sticks.

### What is deliberately refused

- **Geofence and rally uploads** (`MAV_MISSION_TYPE_FENCE` / `_RALLY`) are refused at the count
  with `MAV_MISSION_UNSUPPORTED`. DJI owns these through FlySafe and WildBridge cannot write them.
  Accepting an upload we would then ignore is the silent-drop failure that makes a plan upload
  dangerous.
- **Anything outside the navigation subset.** Only `NAV_WAYPOINT` and `DO_CHANGE_SPEED` are
  accepted; other commands are refused per item with `MAV_MISSION_UNSUPPORTED`.
- **Out-of-order items** are refused with `MAV_MISSION_INVALID_SEQUENCE` rather than reordered:
  the protocol requests items by index, so an unexpected index means the two ends disagree about
  where they are, and guessing would store a plan neither side intended.

A failed upload leaves the previously stored plan untouched, because incoming items are staged
separately and only promoted on a complete upload.

### Position precision

`COMMAND_LONG` carries `param5`/`param6` as float32. At 46° latitude that is roughly **0.6 m** of
rounding error, which is why MAVLink defines `COMMAND_INT` with the position as int32 degE7. Two
fixes followed from measuring this:

- The ground station sends positional commands (`DO_REPOSITION`) as `COMMAND_INT`.
- `MavlinkCommand` now carries `latitudeDeg`/`longitudeDeg` as **doubles**, so a `COMMAND_INT`
  keeps the precision it was sent with instead of losing it on arrival. (The parser accepted
  `COMMAND_INT` before this, but funnelled the position through a float anyway.)

Verified: `46.5180001` round-trips exactly, where float32 gives `46.518001556`.

### Verified against the real aircraft

Upload handshake, per-item requests, ack, download, and exact round-trip of position and the NaN
heading distinction — all confirmed against a live aircraft, plus 14 JVM unit tests
(`MavlinkMissionProtocolTest`) covering the state machine and the refusal paths.

---

## Ground-station transport selector

`WB_TRANSPORT` lets the Python ground station choose its wire without forking the client, because
the whole surface funnels through exactly two chokepoints — the telemetry dictionary and
`requestSend`:

| Value | Telemetry | Commands |
|---|---|---|
| `http` (default) | TCP 8081, as today | HTTP POST |
| `mavlink` | MAVLink UDP | MAVLink; **anything unsupported is refused, never silently retried over HTTP** |
| `both` | MAVLink | MAVLink where it has an equivalent, HTTP where it does not |

MAVLink-derived telemetry is written under **the same keys the HTTP stream already produces**, so
every getter, every ROS publisher and every script is transport-agnostic. A misspelled
`WB_TRANSPORT` raises rather than defaulting to HTTP — a typo that quietly ran over HTTP would
produce a passing "MAVLink" test result.

### Measured gap

Against a live aircraft, MAVLink mode currently carries **10 of the 47** telemetry keys HTTP
provides, and all 10 agree with HTTP exactly. Still HTTP-only: gimbal attitude, zoom/focal
lengths, thermal and LRF state, detections, WebRTC/streaming state, the battery and time budgets,
`distanceToHome`, `readyToTakeoff`/`takeoffBlockReason`, and the yaw/altitude reach latches.

Two bugs were found by making this comparison rather than by reading code:

1. **Heading convention.** MAVLink reports 0–360, DJI reports −180–180. Unconverted, a consumer
   switching transports would see a 360° jump at north.
2. **The camera's heartbeat clobbered the flight mode.** A WildBridge aircraft heartbeats from
   *two* components — the autopilot, and the camera at `MAV_COMP_ID_CAMERA` with a meaningless
   zero `custom_mode`. Believing whichever arrived last reported `UNKNOWN` for a drone sitting in
   position hold. QGroundControl filters the same way, and for the same reason.

---

## Covering Swarm-Steward's whole command set

The ROS controller drives WildBridge through 27 commands. Before this pass MAVLink covered 12,
and the 15 missing included **all three aborts** — its stop path — which made `WB_TRANSPORT=mavlink`
unusable for it. All 27 are now covered.

The rule was: a standard `MAV_CMD` wherever one genuinely fits, and only then the user range.

| HTTP endpoint | MAVLink form |
|---|---|
| `abortMission`, `abortAll`, `abort/DJIMission` | `DO_SET_MODE` → position hold |
| `enableVirtualStick` | `DO_SET_MODE` → offboard |
| `gotoAltitude` | `CONDITION_CHANGE_ALT` |
| `stick` | `MANUAL_CONTROL` (a message, not a command) |
| `navigateTrajectoryDJINative` | mission upload + `MISSION_START` |
| `drop` | `DO_GRIPPER` (release) |
| `setRTHAltitude`, `setSetting` | `PARAM_SET` |
| `gimbal/rel_pitch`, `gimbal/rel_yaw`, `deactivateManualOverride` | `MAV_CMD_USER_1` + selector |
| `lrf/measure`, `captureTemperature` | `MAV_CMD_USER_2` + selector |

Three decisions are worth recording.

**One abort, not three.** The HTTP surface has three aborts with different scopes. MAVLink
expresses an abort as a mode change, which has one meaning, so all three arrive as position hold
and the aircraft does the *union* of the three — stop the PID loops, neutralise the sticks, leave
virtual stick, end any DJI wayline. Widening an abort is the only direction it is safe to be wrong
in.

**Two user commands, not eight.** The residue that has no portable equivalent is carried on
`MAV_CMD_USER_1` (payload aiming) and `USER_2` (payload sensing), each with a selector in `param1`,
rather than spreading across the user range. A ground station that does not know WildBridge simply
never sends them.

**Read commands return their reading.** `COMMAND_ACK.result_param2` exists for exactly this, so the
rangefinder's distance (centimetres) and the thermal spot temperature (hundredths of a degree) come
back in the ack instead of forcing a caller to poll telemetry and hope it is looking at the right
sample. This also meant the ground station had to start *waiting* for acks rather than optimistically
reporting acceptance — so every command now reports its true outcome.

### Writable parameters

`PARAM_SET` is now handled, against an **allowlist of one** (`WB_RTH_ALT`). Most of the published
list is read-only by nature: PID gains belong to the control profile, and the PX4 compatibility
parameters are constants that exist only to satisfy QGroundControl's setup checks. Writing those
would either do nothing or quietly change flight behaviour from a settings dialog. The aircraft
answers every write with a `PARAM_VALUE` carrying what the parameter *now* holds, which is how a
refused write is detected — verified: a write to `WB_DIST_KP` comes back `DENIED` with the old value.

**A caveat, found by testing.** `WB_RTH_ALT` reads `-1` on a real aircraft, because DJI's
`KeyGoHomeHeight` listener only fires on change and nothing seeds it — `.get(default)` reads
KeyManager's cache without triggering a fetch. This is pre-existing and affects the HTTP surface
identically (`GET /config/settings` also reports `-1`). The ground station therefore reports such a
write as `ACCEPTED (unverified: the aircraft does not report WB_RTH_ALT)` rather than either
claiming success or rejecting a write that probably took.

### Stopping a mission — a blocker found before the field

A running plan could only be stopped by `MISSION_CLEAR_ALL`. Pressing **Land** or **RTL** in a
ground station would land the aircraft, and the sequencer — which only watches the reach latch —
would then issue the next leg and fly it away again. Land, RTL, reposition, yaw and abort now all
supersede a running plan. The flight gate moved onto the activity in the same pass: a gate that
only one of the two sinks consulted is a hole, not a gate.

`MAV_CMD_DO_PAUSE_CONTINUE` is still refused. Land/RTL is the stop.

### Fleets need one port per aircraft

The MAVLink telemetry source filters on the aircraft's address, because several aircraft streaming
to one UDP port would be folded into a single telemetry dictionary — which would look like one
drone teleporting between positions rather than like an error. Filtering is not sufficient on its
own: only one socket receives a given UDP port's packets, so **each aircraft must be configured
with its own ground-station port** (`WB_MAVLINK_PORT` on the ground side, `wb_mav_0_port` on the
aircraft).

---

## Closing the telemetry and completion gaps

Counting what the ROS node publishes rather than what the client exposes changed the picture:
36 keys published, 15 carried. The 21 missing were not one problem but two.

### Completion is a command-protocol matter, not a telemetry one

`waypointReached`, `yawReached` and `altitudeReached` are not state — they answer "did the command
I sent finish?". MAVLink answers that in the command protocol: a long-running command is
acknowledged with `MAV_RESULT_IN_PROGRESS`, and acknowledged **again** when it completes.

That is a better fit than the reach latches, because the protocol correlates the completion to the
request: the stale-latch race that the seq numbers exist to work around cannot arise when the
answer is addressed to the question. `DO_REPOSITION`, `CONDITION_YAW` and `CONDITION_CHANGE_ALT`
now report this way, which is what makes a plain goto reportable at all — `MISSION_ITEM_REACHED`
is emitted only by the mission sequencer, so before this a single goto had no arrival report and a
caller waiting on one waited forever.

**The completion must not block the caller.** The first implementation waited for the final ack
inside `send()`, which is correct-looking and wrong: the HTTP contract is that a goto returns a seq
immediately and the caller polls the latch, so blocking until arrival stalls a ROS callback for the
whole flight. Testing it against the aircraft hung for the full timeout, which is how it was found.
The completion is now picked up on a background thread and raises the same `waypointReached` /
`waypointSeq` keys the HTTP surface exposes, so consumers are unchanged. The seq is captured when
the command is sent rather than read when the latch rises — another command may have been issued in
between, and reporting this arrival under that command's number is exactly the confusion the seq
prevents.

Replies are read by one reader thread and handed to waiters, because several coexist: a command
waiting for its first ack, and any number of completion watchers waiting for the second. Two
threads calling `recvfrom` on one socket would each swallow what the other was waiting for.

### The rest, sorted by what MAVLink actually offers

- **Standard messages that already existed.** Gimbal attitude as `GIMBAL_DEVICE_ATTITUDE_STATUS`
  (285, from the gimbal v2 capability the camera component already advertises), and the laser
  rangefinder's range as `DISTANCE_SENSOR` (132) — gated on a lock, because a rangefinder
  reporting a stale range is worse than one reporting nothing.
- **Computed on the ground.** `distanceToHome` is a function of two positions already on the wire.
  Defining a message to save a square root would have been the wrong trade. It is also now *more*
  correct than the HTTP surface, which computes a distance to a home point of (0, 0) before one is
  set and reports 2,559 km.
- **A single dialect message.** `wildbridge.xml` defines one `WILDBRIDGE_STATUS` (42100) carrying
  the genuine residue: the authority latch, takeoff readiness and block reason, DJI's four
  smart-return budgets, the three focal lengths, and the rangefinder's geo-referenced target. One
  message rather than eight `MAV_CMD_USER` slots means a ground station that does not know
  WildBridge ignores exactly one id and loses nothing it understood.

The ground station decodes that one message by hand rather than shipping a generated dialect,
which would put a build step and a second copy of the definition into every consumer. A test
regenerates the definition from the XML with mavgen and asserts the layout still matches, so the
XML is the source of truth in practice and not only in principle.

### Two bugs the comparison caught

**Gimbal field order.** MAVLink packs fields largest-first, so `failure_flags` (u32) precedes
`flags` (u16) despite being declared after it. Getting that backwards produced a frame with a valid
checksum that decoded into a denormal float — the same failure mode as the `EXTENDED_SYS_STATE`
message-id bug earlier in this work: valid MAVLink, silently meaningless.

**Joint gimbal attitude is a rotation, not a subtraction.** DJI reports the gimbal's attitude in
the world frame and its angle relative to the aircraft; MAVLink's message carries only the first.
The second is recovered by composing quaternions (`q_body⁻¹ ⊗ q_world`) rather than subtracting
euler angles, which is only correct while the aircraft is close to level — precisely when it does
not matter. **The sign is calibrated against a single static sample** and is the one field here
that still needs confirming in flight against the HTTP stream.

### Where this leaves the count

MAVLink now carries **27 of the 47** telemetry keys, and 25 of the 27 agree exactly with HTTP
against a live aircraft. Of the two that differ, `distanceToHome` is HTTP being wrong, and
`gimbalJointAttitude` is the sign above. What remains uncarried is the HTTP-only surface —
detections, WebRTC and streaming state, phone location, the battery thresholds — none of which the
flight stack needs.
