# GroundStation Python Quality Refactor Plan

This note captures the first baseline pass for Python and ROS GroundStation code and proposes the same test-first, refactor, re-check loop used for the Android quality work.

## Scope

Treat these areas as separate streams:

- Plain GroundStation scripts: `Python/wildbridge_groundstation/dji_client.py` and `Python/mavlink_proxy.py`
- ROS 2 packages: `ROS/dji_controller`, `ROS/wildbridge_mavros`, `ROS/drone_videofeed`, and launch files under `ROS/wildview_bringup`
- Video test webapp: `video_test/webapp/server.py`

The first pass found 29 Python files and about 5,900 total lines. Largest files are:

- `ROS/wildbridge_mavros/wildbridge_mavros/dji_interface.py` at about 1,300 lines
- `Python/mavlink_proxy.py` at about 850 lines
- `ROS/dji_controller/dji_controller/submodules/dji_interface.py` at about 730 lines
- `ROS/wildbridge_mavros/wildbridge_mavros/mavros_bridge.py` at about 630 lines
- `ROS/dji_controller/dji_controller/controller.py` at about 500 lines
- `video_test/webapp/server.py` at about 410 lines

## Initial Baseline

Current checks run from the repository root:

```sh
python3 -m compileall -q GroundStation/Python GroundStation/ROS GroundStation/video_test/webapp
```

Current result:

- Syntax compilation passed before refactoring and still passes after each slice.
- The repository now has `pyproject.toml`, `.pre-commit-config.yaml`, and `scripts/check_radon_complexity.py`.
- The local `.venv` has the Python quality toolchain needed for Ruff, pre-commit, Radon, mypy, Bandit, and pytest.
- The strict Radon gate requires B-or-better complexity. C/D/E/F blocks fail the check.
- Current strict Radon result after canonical client extraction: A: 337, B: 27, C: 0, D: 0, E: 0, F: 0.
- GroundStation pytest suite: 32 passed.

## Main Issues To Fix

### 1. Duplicate DJI Interface Implementations

Primary files:

- `Python/wildbridge_groundstation/dji_client.py`
- `ROS/dji_controller/dji_controller/submodules/dji_interface.py`
- `ROS/wildbridge_mavros/wildbridge_mavros/dji_interface.py`

Why this matters:

The HTTP/TCP/discovery client logic appears in multiple places. Fixes to telemetry parsing, discovery, endpoint URLs, timeout behavior, and error handling can drift between the plain script path and the ROS packages.

Test first:

- Add characterization tests for discovery response parsing.
- Add tests for command endpoint URL construction.
- Add tests for telemetry JSON parsing and default values.
- Add tests around socket shutdown behavior using fakes, not real drones.

Refactor loop:

1. Extract pure parsing and URL-building helpers.
2. Make both ROS and non-ROS callers use the shared helpers.
3. Only then consider consolidating the duplicated classes or creating a small shared package.
4. Re-run Ruff, tests, and compile checks after each slice.

### 2. Large Bridge/Controller Classes

Primary files:

- `Python/mavlink_proxy.py`
- `ROS/wildbridge_mavros/wildbridge_mavros/mavros_bridge.py`
- `ROS/dji_controller/dji_controller/controller.py`

Why this matters:

These files mix transport setup, message mapping, mission state, telemetry publishing, command handling, logging, and runtime loops. They are hard to test because pure conversion logic is embedded inside ROS/MAVLink/socket code.

Test first:

- Add tests for DJI-to-MAVLink mode mapping.
- Add tests for DJI-to-MAVROS mode mapping.
- Add tests for mission item conversion and waypoint filtering.
- Add tests for telemetry-to-message field mapping where this can be done without importing ROS.

Refactor loop:

1. Extract pure mapping tables and conversion functions.
2. Move mission bookkeeping into small data objects or helper modules.
3. Keep ROS publishers/services and MAVLink sockets as thin adapters.
4. Re-run tests before and after each extraction.

### 3. Runtime Error Handling And Logging

Primary areas:

- Bare `except:` blocks in DJI interface code
- Broad `except Exception` paths in telemetry, video server, and MAVLink loops
- Many direct `print()` calls in long-running services

Why this matters:

GroundStation code runs network and flight-control-adjacent paths. Silent failures can hide broken telemetry, failed command delivery, or socket cleanup bugs.

Test first:

- Add tests for retry/timeout decisions using fake sockets or monkeypatched request calls.
- Add tests that failed HTTP requests produce an explicit failure value.
- Add tests that cleanup closes sockets once and tolerates already-closed sockets.

Refactor loop:

1. Replace bare `except:` with specific exceptions.
2. Convert runtime prints to `logging` or ROS logger calls by boundary.
3. Preserve CLI output where scripts are meant to be interactive.
4. Add Bandit only after intentional subprocess/socket patterns are documented or ignored.

### 4. ROS Package Metadata And Existing Linter Tests

Primary files:

- `ROS/dji_controller/setup.py`
- `ROS/drone_videofeed/setup.py`
- `ROS/*/test/test_flake8.py`
- `ROS/*/test/test_pep257.py`

Why this matters:

ROS packages already contain generated ament linter tests, but the current environment cannot execute them. Some setup metadata still has TODO placeholders.

Test first:

- Make the existing ROS linter tests runnable in the project environment.
- Keep copyright checks skipped unless the project decides to add headers consistently.
- Add package-level smoke tests for importable modules where ROS dependencies are available.

Refactor loop:

1. Install or document the ROS/ament linter dependencies.
2. Decide whether Ruff replaces, supplements, or coexists with generated ament flake8/pep257 tests.
3. Replace TODO package metadata with accurate package descriptions and licenses.

### 5. Video Test Webapp

Primary file:

- `video_test/webapp/server.py`

Why this matters:

This service drives video connection diagnostics and writes NDJSON logs. It should stay simple, but its request handling and event logging should be covered before formatting/refactors touch it.

Test first:

- Add tests for event serialization.
- Add tests for request routing with the stdlib HTTP server handler isolated where possible.
- Add tests for telemetry-trigger payloads without requiring live drones.

Refactor loop:

1. Extract event/log helpers first.
2. Extract drone discovery/connection decisions from request handlers.
3. Keep the stdlib server entrypoint small and direct.

## Proposed Work Order

1. Keep the current Python quality config green: Ruff lint, Ruff format, strict Radon, mypy, Bandit, and pytest.
2. Add characterization tests for each new refactor slice before changing runtime behavior.
3. Run the loop: test, refactor one narrow area, format/lint, test again.
4. Prefer extracting pure helpers before changing ROS, MAVLink, or socket-bound code.
5. Keep the Radon threshold strict: B-or-better only, with no C/D/E/F baseline allowances.

## Candidate First Slice

Completed first slice: DJI telemetry and discovery helpers, because they are duplicated and can be tested without ROS, MAVLink, or live drones:

1. Extracted `parse_discovery_response`, `build_command_url`, and `parse_telemetry_line` helpers.
2. Added pytest tests for valid, invalid, and partial inputs.
3. Wired the plain Python client path to the helpers.
4. Refactored C-or-worse complexity blocks across the plain DJI interface, ROS DJI interfaces, MAVLink proxy, MAVROS bridge, launch helpers, and video test webapp.
5. Re-ran compileall, Ruff, pytest, strict Radon, mypy, Bandit, and pre-commit successfully.

Completed second slice:

1. Promoted the DJI helpers into the `wildbridge_groundstation.dji_helpers` package while keeping `wildbridge_dji_helpers.py` as a compatibility wrapper.
2. Added shared tests for legacy ROS discovery tuple parsing, subnet scan candidate generation, and telemetry chunk parsing.
3. Wired the plain and ROS DJI interfaces to the shared URL, discovery, subnet, and telemetry chunk helpers.
4. Added Docker `PYTHONPATH` configuration so the GroundStation runtime can import the shared helper package.
5. Extracted pure video-grid event helpers for NDJSON entries, SSE messages, and discovery parsing, then covered them with pytest tests.

Completed third slice:

1. Split shared DJI discovery response parsing into decode, prefix, payload, and fallback helpers.
2. Kept the existing discovery helper tests green and reduced the helper module's discovery parser from B complexity to A complexity.

Completed fourth slice:

1. Extracted pure MAVLink helper functions for armed-state mapping, heartbeat base-mode flags, GPS fix type, ground speed, and climb-rate conversion.
2. Added pytest coverage for those MAVLink decisions without importing `pymavlink` or opening sockets.
3. Wired `Python/mavlink_proxy.py` to the helpers, reducing heartbeat-loop complexity while preserving MAVLink message boundaries in the proxy.

Completed fifth slice:

1. Added the canonical `wildbridge_groundstation.dji_client.DJIInterface` client for common HTTP command and TCP telemetry behavior.
2. Added tests for discovery normalization, config name lookup, telemetry buffering, command URL posting, command formatting, and socket/thread shutdown.
3. Replaced the old plain `djiInterface.py` implementation with shared-client usage.
4. Replaced `ROS/dji_controller/.../dji_interface.py` with a package-local discovery adapter around the shared client.
5. Replaced the MAVROS local client class with a shared-client adapter while preserving its richer multicast/mDNS/cache discovery code.

## Current Green Checks

Run from the repository root:

```sh
.venv/bin/pre-commit run --all-files
```

Current result:

- Ruff lint: passed.
- Ruff format: passed.
- Radon complexity, B-or-better only: passed.
- mypy gradual typing: passed.
- Bandit security scan: passed.
- GroundStation pytest suite: 32 passed.