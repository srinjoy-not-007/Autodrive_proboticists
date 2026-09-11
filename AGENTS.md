# Repository Guidelines

## Project Structure & Module Organization

`src/pit_crew/` contains ROS 2 Python nodes; shared vehicle constants live in `vehicle.py`. Launch files and parameters are in `src/launch/` and `src/config/`; container utilities are under `src/tools/`. Run offline map, raceline, and log analysis from `tools/` on the host. Tracked artifacts belong in `data/maps/` and `data/racelines/`; bags, logs, and screenshots are ignored. `scripts/` contains development workflows, while `submission/` defines the competition image.

## Build, Test, and Development Commands

- `./scripts/pull_images.sh`: fetch the official simulator and devkit images.
- `./scripts/run_sim.sh` and `./scripts/run_devkit.sh headless`: start simulation and the ROS bridge. Use a windowed simulator for valid LiDAR.
- `./scripts/smoke_test.sh`: exercise sensor flow and actuation end to end.
- `./scripts/shell_devkit.sh`: open a ROS-configured shell in the running devkit.
- `colcon build --packages-select pit_crew --symlink-install`: build the package inside the devkit container.
- `colcon test --packages-select pit_crew && colcon test-result --verbose`: run registered ROS tests and linters.
- `.venv/bin/python tools/validate_laps.py`: analyze recorded lap results on the host.

Run `./scripts/stop_run.sh` after driving to terminate nodes and explicitly zero latched controls.

## Coding Style & Naming Conventions

Use four-space indentation and PEP 8 for Python. Name modules, functions, nodes, topics, and parameters with `snake_case`; use `UPPER_CASE` for constants. Keep entry points synchronized with `src/setup.py`. Bash scripts should quote expansions and be executable. Prefer YAML or ROS parameters over track-specific controller constants. The package declares `ament_flake8` and `ament_pep257`; document public APIs concisely.

## Testing Guidelines

Tests live in two places: `tests/` holds host-side pytest for the offline tools (`.venv/bin/python -m pytest tests`), and `src/test/` holds pure-Python and ROS-node tests that run inside the devkit container. Add new tests beside the code they cover, named like `test_localization.py`. For driving changes, run offline analysis, the smoke test, then a measured simulator run. Record lap time, collisions, and configuration in `docs/STATUS.md`; label estimates. Never use restricted simulator pose, odometry, collision, lap, or TF topics as controller inputs.

## Commit & Pull Request Guidelines

Recent commits use imperative, sentence-style subjects describing the result, for example `Audit what is tuned to Porto`. Keep commits focused and include updated documentation when behavior or measurements change. Pull requests should explain the motivation, list exact validation commands, identify tested map/controller settings, and summarize measured regressions or gains. Link relevant issues and attach plots or screenshots when geometry, trajectories, or simulator UI changes.

## Competition Constraints

Do not modify the upstream `autodrive_roboracer` devkit package or simulator. Keep team code in the separate `pit_crew` package, and ensure submission startup remains in `submission/autodrive_devkit.sh`, not shell initialization files.
