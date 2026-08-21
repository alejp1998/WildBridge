# GroundStation Python Quality Checks

Run these commands from the repository root.

## Current Baseline

The only check that currently runs in the system Python environment without extra tooling is syntax compilation:

```sh
python3 -m compileall -q GroundStation/Python GroundStation/ROS GroundStation/video_test/webapp
```

Initial result on 2026-05-29:

- `python3 --version`: Python 3.14.4
- `compileall`: passes
- `python3 -m pytest GroundStation/ROS/dji_controller/test GroundStation/ROS/drone_videofeed/test -q`: blocked, `pytest` is not installed
- `python3 -m pip show ...`: blocked, this Python has no `pip` module
- `ruff`, `black`, `pre-commit`, `flake8`, `pydocstyle`, and `colcon` are not on `PATH`

## Proposed Tooling Loop

Mirror the neighboring `dialogue-swarm` quality setup, adjusted for ROS/GroundStation paths:

```sh
ruff format --check GroundStation
ruff check GroundStation
python scripts/check_radon_complexity.py
mypy
bandit -r GroundStation -ll --skip B101
pytest GroundStation -q
pre-commit run --all-files
```

Recommended first configuration:

- Ruff line length: 100
- Ruff target version: Python 3.10, because ROS Humble commonly runs on Python 3.10 even if the local workstation is newer
- Ruff rules: `E`, `W`, `F`, `I`, `B`, `UP`, `N`, `C4`, `SIM`, `C90`, `RUF`
- Complexity threshold: Radon and Ruff require B-or-better functions; C/D/E/F blocks fail
- Mypy starts narrow and gradual, with missing imports ignored for ROS/DJI/MAVLink dependencies
- Bandit scans `GroundStation`, skipping `B101` for tests/assertions

## Expected Pre-Commit Hooks

Use the same shape as `dialogue-swarm`:

- Ruff lint with safe fixes
- Ruff format
- Radon complexity gate
- Mypy gradual type check
- Bandit security scan
- Manual pytest hook for the full GroundStation suite

Keep pre-commit focused on Python files under `GroundStation` so Android/Kotlin quality remains owned by the Android Gradle checks.