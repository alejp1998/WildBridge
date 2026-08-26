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

All changes compile cleanly (`:sample:compileCurrentDebugKotlin` → BUILD SUCCESSFUL).

**Stage 1 — foundation (zero behaviour change):**

- `mavlink/MavlinkCommandSink.kt` — the 5 sink methods now return `CommandResult`; added the
  `CommandResult` data class; `MavlinkCommandOutcome` enum kept as-is.
- `mavlink/MavlinkTelemetryEndpoint.kt` — `executeCommand` now builds `CommandResult` and returns
  `result.mavResult`.
- `WildBridgeDefaultLayoutActivity.kt` — the 5 `MavlinkCommandSink` implementations and the
  `awaitAction` helper return `CommandResult`; added `import ...mavlink.CommandResult`.
- New test `src/test/java/dji/sampleV5/aircraft/mavlink/CommandResultTest.kt` (JUnit 4).

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
- Test: `src/test/java/dji/sampleV5/aircraft/mavlink/MavlinkSystemIdTest.kt`.

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


