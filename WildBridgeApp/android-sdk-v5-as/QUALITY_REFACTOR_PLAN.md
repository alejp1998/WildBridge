# WildBridge Android Quality Refactor Plan

This note explains the main quality issues found by Spotless/ktlint, Detekt, and Android Lint, why they matter, and how we should fix them without mixing our code with DJI/vendor code.

Run commands from `WildBridgeApp/android-sdk-v5-as`.

## Scope

We should treat these as two separate streams:

- WildBridge-owned code: `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/` custom packages such as `webrtc`, `edge`, `controller`, `formation`, `logger`, `server`, and `WildBridgeDefaultLayoutActivity.kt`.
- DJI/vendor code: `../android-sdk-v5-uxsdk/` and DJI sample areas that we mostly inherit. We still report on them, but we do not spend refactor effort there unless we intentionally maintain a local patch.

The important tasks are:

```sh
./gradlew qualityWildBridge
./gradlew qualityDji
./gradlew :sample:testDebugUnitTest
./gradlew :sample:compileDebugKotlin
```

## Current Baseline

The first stable WildBridge loop now works:

```sh
./gradlew --continue :sample:spotlessKotlinCheck :sample:compileDebugKotlin qualityWildBridge
```

Focused refactors completed so far:

- `SdpUtils`: now pure Kotlin, covered by unit tests, and no longer appears in the WildBridge Detekt findings.
- `AdaptiveFrameRatePolicy`: extracted from `WebRTCStreamer`, covered by unit tests, and removed the `maybeAdaptFrameRate` complexity findings from `WebRTCStreamer`.
- `LetterboxTransform`: extracted from `YoloTfliteDetector`, covered by unit tests, and removed the detector's coordinate-mapping Detekt findings.
- `PID`: anti-windup limit checks are now explicit, covered by unit tests, and no longer appear in the WildBridge Detekt findings.
- `MockTelemetryOrigin`: extracted from `TelemetryProvider`, covered by unit tests, and removed mock-location condition complexity from telemetry setup.
- `shouldSwitchToDroneVideoSource`: extracted from `WildBridgeDefaultLayoutActivity`, making aircraft connection source switching explicit and removing its condition complexity finding.
- `TelemetryProvider.captureMetadata`: split into mock and cached metadata builders, removing the telemetry long-method finding.
- `WebRTCStreamer.startWhip`: split publisher reuse, listener wiring, and source-loss checks into helpers; local IP lookup was flattened and now catches `SocketException`.
- `WebRTCStreamMetrics.compactLabel`: split the status label into small helpers and removed the metrics file's Detekt formatting findings.
- `EdgeDetectionController`: introduced `EdgeDetectionConfig`, shared frame-admission logic, and `runCatching` inference paths; the controller is now absent from the WildBridge Detekt report.

Reports to inspect:

- WildBridge Detekt: `build/reports/detekt/wildbridge.html`
- DJI/vendor Detekt: `build/reports/detekt/dji.html`
- Sample Android Lint: `../android-sdk-v5-sample/build/reports/lint-results-debug.html`
- UXSDK Android Lint: `../android-sdk-v5-uxsdk/build/reports/lint-results-debug.html`

## Main Issues To Fix

### 1. Very Large Activity

Primary file:

- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/WildBridgeDefaultLayoutActivity.kt`

Main symptoms:

- `LargeClass`
- `TooManyFunctions`
- long methods
- complex methods
- many long lines
- broad responsibilities in one Android Activity

Why this matters:

The Activity currently owns UI state, video controls, telemetry display, HTTP/control endpoints, edge detection state, network discovery, preferences, and streaming coordination. When one file owns that much, every change becomes risky because it is hard to know what else will be affected. It also makes testing almost impossible, because most behavior is tied directly to Android lifecycle and UI objects.

What we should do:

- Extract pure or mostly pure services first.
- Move embedded HTTP/control routing out of the Activity.
- Move edge detection UI/state coordination into a small controller class.
- Move network/IP/discovery helpers into a separate utility or service.
- Keep the Activity as the UI wiring layer, not the owner of all behavior.

Good first extraction target:

- the local HTTP/control server and `handlePostRequest` logic, because it is both long and complex and can become testable without the full Android UI.

Done already:

- Aircraft connection source-switching logic was named as `shouldSwitchToDroneVideoSource`, reducing condition complexity in `applyAircraftConnectionState`.

### 2. WebRTC Flow Complexity

Primary files:

- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/webrtc/WebRTCStreamer.kt`
- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/webrtc/WhipPublisher.kt`
- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/webrtc/SharedDJIFrameSource.kt`
- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/webrtc/SharedPhoneCameraFrameSource.kt`

Main symptoms:

- return-count findings
- long methods
- nested frame-handling logic
- too many throws in publishing flow
- wildcard imports
- long lines

Why this matters:

Video streaming is one of the most important runtime paths in the app. Complexity here can cause subtle bugs: dropped frames, stuck listeners, incorrect source switching, poor recovery after network errors, or hard-to-debug WHIP publishing failures. Smaller components make it easier to test decisions such as frame-rate adaptation, SDP munging, and publishing retries.

What we should do:

- Keep pure SDP and media-option logic tested with JVM tests.
- Extract frame-rate adaptation decisions from `WebRTCStreamer` into a small policy class.
- Split WHIP publishing into request building, response parsing, and peer-connection lifecycle pieces.
- Reduce early returns in frame-source classes by extracting small guard helpers.

Done already:

- `SdpUtils` was refactored and covered by `SdpUtilsTest`.
- Adaptive frame-rate decisions were extracted into `AdaptiveFrameRatePolicy` and covered by `AdaptiveFrameRatePolicyTest`.
- Mock telemetry origin validation was extracted into `MockTelemetryOrigin` and covered by `MockTelemetryOriginTest`.
- Metadata capture now delegates to separate mock and cached metadata builders.
- WHIP startup now delegates publisher reuse, callbacks, and source-loss checks to named helpers.
- Local IP lookup now delegates address scanning to a helper and catches `SocketException` specifically.
- WebRTC stream metric labels now build from focused helper methods instead of one long interpolated string.

### 3. Edge Detection Pipeline Complexity

Primary files:

- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/edge/EdgeDetectionController.kt`
- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/edge/YoloTfliteDetector.kt`

Main symptoms:

- return-count findings
- loop jump complexity
- long lines
- image conversion and inference responsibilities mixed in one area

Why this matters:

Edge detection touches camera frames, threading, model inference, UI overlays, and metrics. Bugs here can hurt app performance or block video processing. The code also needs to be understandable because model formats and thresholds may change as the detection model improves.

What we should do:

- Separate frame gating from inference execution.
- Extract image conversion helpers from detection orchestration.
- Add tests for coordinate conversion and post-processing where possible.
- Keep Android/image APIs at the boundary and pure math in testable functions.

Done already:

- Letterbox coordinate mapping was extracted into `LetterboxTransform` and covered by `LetterboxTransformTest`.
- YUV conversion helpers were moved out of `YoloTfliteDetector`, reducing the detector's function count and removing its current Detekt findings.
- `EdgeDetectionConfig` now groups model, label, source, and confidence settings for controller construction.
- Frame admission for NV21 and YUV inference now flows through the same helper, removing duplicate throttling and busy checks.

### 4. Drone Control And Formation Logic

Primary files:

- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/controller/DroneController.kt`
- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/controller/FormationController.kt`
- `../android-sdk-v5-sample/src/main/java/dji/sampleV5/aircraft/controller/PID.kt`

Main symptoms:

- very large controller object
- return-count findings
- wildcard imports
- unused private formation/collision helpers
- long lines
- constants that can be `const val`

Why this matters:

This is flight-control-adjacent code. Even if the app is not directly flying autonomously in every path, this logic is safety-sensitive and should be easier to reason about than normal UI code. Large controller objects also make it hard to isolate calculations from SDK side effects.

What we should do:

- Identify pure calculations and move them into tested Kotlin classes.
- Remove unused private formation/collision code if it is truly dead, or wire it intentionally if it is planned behavior.
- Keep DJI SDK calls in adapter-like boundaries.
- Add tests around PID, target-position math, and any collision-risk calculation before changing behavior.

Done already:

- PID output-limit and anti-windup behavior is covered by `PIDTest`.
- `PID` now uses the package matching its source path, and the old package import was removed from `DroneController`.

### 5. Silent Or Generic Error Handling

Primary areas:

- `WildBridgeDefaultLayoutActivity.kt`
- `TelemetryServer.kt`
- `WhipPublisher.kt`
- `SharedDJIFrameSource.kt`
- `WildBridgeFlightLogger.kt`
- controller classes

Main symptoms:

- empty catch blocks
- generic caught exceptions
- generic thrown exceptions
- swallowed exceptions

Why this matters:

Silent failures make field debugging painful. In drone/video workflows, a failure often happens on-device, under network pressure, or while interacting with DJI SDK state. If we swallow exceptions without logging enough context, we lose the only clue for diagnosing production issues.

What we should do:

- Replace empty catches with explicit logging or a deliberate ignored-result helper.
- Catch narrower exception types when possible.
- Convert repeated error patterns into small helpers.
- Make user-visible failures clear where the operator can act on them.

### 6. Android Lint Findings

Primary reports:

- `../android-sdk-v5-sample/build/reports/lint-results-debug.html`
- `../android-sdk-v5-uxsdk/build/reports/lint-results-debug.html`

Why this matters:

Android Lint catches platform-specific issues that Detekt does not: deprecated APIs, resource problems, permissions, lifecycle concerns, manifest issues, and performance pitfalls. Some warnings are inherited from DJI/vendor code, but WildBridge-owned lint findings should be reviewed because they may become runtime or compatibility problems.

What we should do:

- Keep `:uxsdk:lintDebug` report-only for vendor awareness.
- Use `:sample:lintDebug` as the actionable report.
- Fix WildBridge-owned lint issues when they touch runtime correctness, permissions, lifecycle, or compatibility.
- Avoid spending time on cosmetic vendor lint unless we maintain that patch.

## Suggested Order Of Work

### Phase 1: Keep The Loop Green

Before and after each refactor:

```sh
./gradlew :sample:compileDebugKotlin
./gradlew :sample:testDebugUnitTest
./gradlew detektWildBridge
```

If changing Android resources or UI behavior:

```sh
./gradlew :sample:lintDebug
```

### Phase 2: Fix Small Pure Utilities First

These are low-risk and build testing habits:

- SDP utilities
- frame-rate adaptation policy
- telemetry formatting
- coordinate conversion
- PID/math helpers

### Phase 3: Extract Medium Components

Good next targets:

- WHIP request/response handling from `WhipPublisher`
- frame listener selection from `SharedDJIFrameSource`
- edge detection metrics/frame gating from `EdgeDetectionController`

### Phase 4: Split The Large Activity

Once enough behavior has tests, extract from `WildBridgeDefaultLayoutActivity`:

- HTTP/control API routing
- edge detection UI coordination
- video-source state coordination
- network discovery/IP helpers
- telemetry display/cache updates

This should be done in multiple small PRs. A giant Activity split without tests would be hard to review and easy to break.

## Definition Of Better

Code quality is improving when:

- `:sample:compileDebugKotlin` stays green.
- `:sample:testDebugUnitTest` stays green and grows around refactored logic.
- WildBridge Detekt findings decrease for the file being touched.
- New code does not add untested pure logic inside Android Activities.
- DJI/vendor findings remain separated from WildBridge-owned findings.
- Refactors reduce responsibilities, not just line counts.

The goal is not to make every report zero immediately. The goal is to make the most important code easier to test, easier to review, and less risky to change.
