# AGENTS.md — WildBridge

WildBridge is an open-source Android ground-station app (Kotlin + DJI Mobile SDK V5) that turns a DJI Remote Controller into a networked drone server: HTTP command API, TCP telemetry, WHIP/WHEP video publishing, and auto-discovery. It is paired with a Python GroundStation library, ROS 2 Humble packages, and a Docker MediaMTX/video-test stack. This file tells coding agents how the repo is laid out and how to work in it safely.

## Project Overview

- **Android app** (Kotlin, DJI MSDK V5.18): `WildBridgeApp/android-sdk-v5-as/` — the `:sample` module is the app; `:uxsdk` is stock DJI UXSDK with a few intentional modifications.
- **Python GroundStation**: `GroundStation/Python/wildbridge_groundstation/` — `dji_client.py` (`DJIInterface`), `dji_helpers.py`, `mavlink_helpers.py`; `wildbridge_dji_helpers.py` is a compatibility shim.
- **ROS 2 Humble**: `GroundStation/ROS/` — `dji_controller`, `drone_videofeed`, `wildview_bringup` (launch files).
- **Video test stack**: `compose.video-test.yaml` — MediaMTX + a browser dashboard in `GroundStation/video_test/webapp/`.
- **Safety model**: two-computer authority. A Safety Computer can seize command authority via the `X-Safety-Token` header; takeover is persistent and only the Safety Computer returns control. Safety-adjacent logic lives in `GroundStation/Python/djiInterfaceSafety.py` and on-device.
- **Key ports on each drone**: HTTP commands `8080`, TCP telemetry `8081`, UDP discovery `30000` (+ mDNS, subnet scan), WHIP/WHEP video via MediaMTX.
- **Supported public video path**: WHIP publish → MediaMTX → WHEP playback. The older direct WebSocket-signaling viewer/server path has been removed from the public app and tooling; do not reintroduce it.

## Coding Agent Workflow

1. Preserve unrelated worktree changes. Read `git status` and the relevant diff before editing a modified file; never reset or overwrite user work.
2. Run checks from the repository root for Python (see Quality Gates), and from `WildBridgeApp/android-sdk-v5-as/` for Gradle work. The persistent terminal keeps its working directory — be explicit about which root a command belongs to.
3. Start from the failing behavior, owning abstraction, nearby test, or exact log entry. Avoid broad refactors until a focused check proves the controlling path.
4. GroundStation Python: keep pure helpers extracted before touching ROS/MAVLink/socket-bound behavior. When changing anything on the MAVLink wire, check it against the HTTP surface with the dashboard's MAVLink tab — every defect in that work so far has been a frame that decoded cleanly and meant the wrong thing, which no compiler or linter sees. Do not hide missing dependencies behind conditional imports or `try/except` import fallbacks (except documented cross-container cases).
5. Make the smallest coherent change, run the narrowest relevant test immediately, then widen validation in proportion to risk.
6. Keep code, comments, and documentation in English.

### Generated and Runtime Files

- Never edit `build/`, `**/outputs/`, `dist/`, caches, `*.apk`, `*.ndjson`, `GroundStation/video_test/logs/`, or other generated/runtime artifacts by hand.
- `local.properties` (Android SDK path + DJI API key) is per-machine and never committed.
- Do not commit deployment secrets, `google-services.json`, keystores, recordings, or runtime state.

## Quick Start

### Python GroundStation

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e GroundStation/Python                # shared client; pulls in requests
pip install -r GroundStation/ROS/requirements.txt   # only if working on ROS bits
pip install pymavlink                               # only for mavlink_listen.py
pytest GroundStation/tests -q
python3 -m compileall -q GroundStation/Python GroundStation/ROS GroundStation/video_test/webapp
```

### Android

```bash
cd WildBridgeApp/android-sdk-v5-as
cp local.properties.example local.properties   # set sdk.dir and AIRCRAFT_API_KEY
./gradlew :app:compileDebugKotlin            # fast validation of Kotlin changes
./gradlew :app:assembleCurrentDebug          # current variant
./gradlew :app:assembleDemoBiomassDebug      # demo/biomass variant
./auto_install_on_connect.sh current --build    # build+install to a connected device
```

### Video test stack

```bash
docker compose -f compose.video-test.yaml up -d --build
# dashboard: http://localhost:8090   MediaMTX WHIP/WHEP: http://localhost:8889
# RTSP: rtsp://localhost:8554  MediaMTX API: http://localhost:9997  ICE UDP: :8189
docker compose -f compose.video-test.yaml down
```

Runtime diagnostics land in `GroundStation/video_test/logs/` (git-ignored).

## Project Structure

| Path | Purpose |
|------|---------|
| `WildBridgeApp/android-sdk-v5-as/` | Android build root (`:sample`, `:uxsdk`); WildBridge additions in `webrtc/`, `formation/`, `controller/`, `server/`, `logger/` packages |
| `WildBridgeApp/android-sdk-v5-sample/` | App source (`com.wildbridge.rc`), navigation graph, WildBridgeDefaultLayoutActivity |
| `GroundStation/Python/wildbridge_groundstation/` | Shared Python helper package (dji_client, dji_helpers, mavlink_helpers, transport) |
| `GroundStation/mavlink/wildbridge.xml` | The WildBridge MAVLink dialect. Source of truth for `WILDBRIDGE_STATUS`; regenerate with mavgen and update the struct, size and CRC_EXTRA in `transport.py` together |
| `GroundStation/Python/djiInterfaceSafety.py` | Safety-authority handling for the two-computer model |
| `GroundStation/ROS/dji_controller/` | ROS package wrapping DJI control |
| `GroundStation/ROS/drone_videofeed/` | ROS package for video feed |
| `GroundStation/ROS/wildview_bringup/` | Launch/config for the ROS stack |
| `GroundStation/tests/` | Pytest suite (`test_dji_client.py`, `test_dji_helpers.py`, `test_mavlink_helpers.py`, `test_video_events.py`) |
| `GroundStation/video_test/` | MediaMTX config + webapp for the video dashboard |
| `scripts/check_radon_complexity.py` | Complexity gate used by pre-commit/CI |
| `compose.video-test.yaml` | MediaMTX + dashboard compose file |
| `verify-integration.sh` | Merge-integrity check for the integration branch (see below) |

## Configuration Notes

- `pyproject.toml` is the source of truth for Ruff (line length 100, Python 3.10 target), pytest paths (`GroundStation/Python` + `GroundStation/video_test/webapp`), mypy scope, and bandit scope.
- Android SDK/API key live in `local.properties` (never committed).
- MediaMTX behavior is defined in `GroundStation/video_test/mediamtx.yml`.

## Testing & Quality Gates

Pre-commit hooks and CI run the same checks; treat a failing local hook as part of finishing the change:

```bash
.pre-commit run --all-files
```

Hooks configured in `.pre-commit-config.yaml`:

- **Ruff lint + format** — scoped to `GroundStation/**.py` (`ruff check --fix`, `ruff format`)
- **Radon complexity** — `python scripts/check_radon_complexity.py`, B-or-better blocks (blocks with cyclomatic complexity ≥ 11 fail)
- **Mypy** — gradual typing over `GroundStation/Python/wildbridge_groundstation` + `wildbridge_dji_helpers.py`
- **Bandit** — `bandit -r GroundStation -ll --skip B101`
- **GroundStation tests** — `python -m pytest GroundStation/tests -q` (manual stage)

Run the manual test hook with:

```bash
pre-commit run groundstation-tests --hook-stage manual
```

Android/Kotlin quality is owned by the Gradle build (`./gradlew :app:compileDebugKotlin` + variant builds), not by the Python hooks.

## Code Conventions

- All code, comments, and docs in English; Python type hints expected.
- Imports at module scope, no conditional-import flags; add new packages to the relevant `requirements.txt`.
- Follow existing patterns when adding GroundStation helpers, ROS nodes, or app pages; register new Android pages in `data/AircraftFragmentPageInfoFactory.kt` and the nav graph.
- Keep generated/runtime artifacts out of Git.
- Safety-critical changes (authority takeover, virtual-stick, control loops, RTH) deserve extra tests and explicit review; never weaken the takeover semantics.

## Commit Attribution

- Do **not** add `Co-Authored-By:` lines for AI assistants to commit messages.
- If acknowledging AI assistance, append a short plain-text comment at the very end of the commit message naming the model that helped, e.g. `AI-assisted by Claude Opus 5.` or `AI-assisted by GitHub Copilot (DeepSeek V4 Pro).` (use the actual model name).
- Other human `Co-Authored-By:` trailers are fine and must be kept.

## Current Branch: `integration/xprize-biomass`

- `verify-integration.sh` verifies the rebuild-forward branch against a pinned-base 3-way merge of the XPRIZE base and the upstream fork. The correct merge base is `7d49349` (immediately before PR #12); letting git choose a base walks back and manufactures phantom conflicts that can silently drop XPRIZE changes.
- MAVROS-related paths (`wildbridge_mavros`, `mavlink_proxy`) are intentionally removed — they are excluded from the verify script's diff and should not be "restored" during merges.
- When touching the merge/verify workflow, run `./verify-integration.sh` and review the printed differing paths.
