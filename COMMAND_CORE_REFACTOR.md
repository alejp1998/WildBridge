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

## 4. What remains (Stage 2 onward, not started)

1. Extend `MavlinkCommandSink` to cover the **four** gimbal operations the HTTP routes actually
   do — today the sink only has `setGimbalPitchYaw` (absolute). The HTTP routes are:
   `/send/gimbal/pitch` (absolute), `/send/gimbal/yaw` (absolute),
   `/send/gimbal/rel_pitch` (relative), `/send/gimbal/rel_yaw` (relative). Each builds a different
   `GimbalAngleRotation` (mode + per-axis enable flags). Then re-route them through the sink.
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


