# Issue #168 validation record

Implementation base: `167-KS/sensor-messages`, `50afd37`.
Working branch: `168-KS/cone-slam`.
Target: Jetson Orin Nano, 25 W; existing camera/LiDAR/planning workload.
Full-track completion remains **unverified**. Unit tests are not evidence of a
successful real lap or measured detector/map accuracy.

## Provisional targets, set before held-out testing

These are engineering starting targets, not achieved results or agreed race limits.
Track clearance and stopping-distance measurements may require tighter targets.
Freeze the selected configuration before evaluating the held-out repeat pass.

| Measurement | Initial target |
|---|---|
| Complete track | One supervised lap without collision, manual recovery or wrong-section routing |
| Landmark position RMSE against independent survey | <= 0.20 m |
| Vehicle XY error p95 / maximum | <= 0.25 m / 0.50 m |
| Heading error p95 | <= 5 degrees |
| Confirmed ID switches on held-out repeat pass | 0 |
| Accepted false loop closures / wrong restart placements | 0 |
| True-loop drift reduction | >= 50%, with final error meeting pose target |
| Timestamp-to-map latency p95 | <= 200 ms at effective 14 Hz camera / 8 Hz LiDAR |
| Map/status publication | 10 Hz; availability false on stale input |
| Optimization p95 | <= 250 ms, no sustained solve-budget rejection |
| SLAM resident memory | <= 512 MiB; bounded growth through full run |

## Development checks

Core tests exercise capture interpolation, transient expiry/dropout, duplicate and
cross-sensor evidence, reordered adjacent same-color cones, delayed input, color
conflict/image labels, unique/ambiguous/degenerate constellations, joint synthetic
loop drift reduction, bounded solver commits, saved-map restart from shifted and
rotated odometry, reference immutability, map validation and capacity exhaustion.

Commands from the repository directory:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOG_DIR=/tmp/cone_slam_ros_logs ROS_DOMAIN_ID=168 PYTHONPATH=src/navigation/cone_slam:$PYTHONPATH ../venv/bin/python -m pytest src/navigation/cone_slam/test -q
../venv/bin/ruff check src/navigation/cone_slam
../venv/bin/ruff format --check src/navigation/cone_slam
../venv/bin/mypy --config-file pyproject.toml src/navigation/cone_slam/cone_slam src/navigation/cone_slam/test src/navigation/cone_slam/launch
make package rc_interfaces perception_observation cone_slam path_planning launch_pkg
```

Actual results are recorded below after the final checks. Do not infer that unlisted
recorded or hardware acceptance criteria passed.

## Reconnaissance of available recordings

Seven local recordings were found: 20261011_145034 (40.9 s), 145349 (10.7 s),
145438 (28.5 s), and 20261024_122251 (32.9 s), 123317 (45.5 s), 123836 (44.9 s),
130513 (58.2 s). Their metadata includes `/odom` but **none contain #167 observation
messages**. Metadata alone does not establish a complete loop, surveyed ground
truth or repeat-pass coverage. Raw sensor replay through the current detectors
would be needed before using these bags for end-to-end evaluation.

## Tomorrow's sequence

1. Record the source commit plus dirty diff, configuration and model hashes,
   extrinsics, track survey/ground truth method, JetPack/ROS versions, power mode
   (`nvpmodel -q`) and thermal/memory/CPU telemetry (`tegrastats`). Do not assume a
   particular numeric nvpmodel mode ID corresponds to 25 W.
2. Confirm exactly one `map -> odom` publisher, smooth `odom -> base_link`, and
   matching sensor/body timestamps and frame names. Measure reference-point bias
   and XY residuals by sensor, range and bearing; replace provisional noise values
   using independent reference measurements.
3. Start standalone SLAM with the planner/controller stopped. Teleoperate one
   complete circuit and a repeat pass, recording `/odom`, `/tf`, `/tf_static`, both
   observation topics, camera calibration/raw streams, `/slam/map`, `/slam/pose`,
   `/slam/status` and power/runtime telemetry. Preserve a held-out repeat pass.
4. Inspect stable IDs through stopped-camera and stopped-LiDAR intervals. Include
   distant LiDAR-only cones, conflicting labels, occlusion, nearby equal-color cones,
   a repeated corner and parallel sections. Do not count out-of-view cones as misses.
5. Save the map and restart at a different position and heading in localization
   mode. Allow multiple distinct observations while teleoperating until a unique
   constellation is established. Confirm reference geometry stays unchanged.
6. Verify map and pose continue with no `/waypoints` publisher, while a separate
   test publisher clears a path, and after that publisher exits. SLAM subscribes to
   no path topic. Confirm pose availability goes false on stale odometry/observations.
7. Replay under the combined perception/planning workload at 25 W. Measure map/pose
   error, ID switches, false landmarks/closures, p50/p95/max latency and peak RSS.
   Compare against the frozen targets; report each failed/blocked/unrun check.
8. Only after mapping/localization and existing safety checks pass, evaluate a
   supervised low-speed lap using the integrated SLAM profile. Wrong-section route
   selection is a planner finding (#170), not evidence to loosen loop-closure gates.

No hardware performance, held-out full-course accuracy, sensor covariance
calibration, or successful autonomous-lap result is claimed by this implementation.

## Actual development results (2026-11-13)

Source base: `50afd373f350a107062981df6d5f94b444e43869`, plus the uncommitted
implementation on `168-KS/cone-slam`. SHA-256 of sorted modified/untracked non-Markdown
file names and bytes (NUL-separated):
`b938d6bf1677b9847b1a42d0d16737bb2761072911db5263dfbde6f3bd7a7b5a`.
Environment: local x86-64 development host, ROS Humble, Python 3.10.12,
NumPy 1.26.4, SciPy 1.15.3, pytest 9.2.0. **Not the Jetson performance environment.**
No detector model or recorded dataset was used for these synthetic checks.

| Check | Actual outcome |
|---|---|
| Build rc_interfaces, perception_observation, cone_slam, path_planning, launch_pkg | PASS, all five packages |
| cone_slam package tests | PASS, 19 tests, 0 failures/skips; 15 core + 4 ROS cases |
| Existing planner unit tests | PASS, 3 tests |
| Planner clang-format, flake8, CMake lint, docstrings, XML | PASS; XML rerun with network access after sandbox DNS failure |
| Planner cppcheck | SKIPPED by existing ament wrapper because installed cppcheck 2.7 is unsupported |
| Ruff check and format for changed/new Python | PASS |
| Mypy for cone_slam core/tests/launch | PASS, 10 source files |
| git diff --check | PASS |
| Installed standalone executable startup/SIGINT shutdown | PASS, 2-second isolated-domain smoke; timeout exit 124 is intentional; no traceback |
| Standalone launch argument resolution | PASS |
| Jetson 25 W combined-workload latency/memory | UNRUN |
| Measured reference-point/covariance calibration | UNRUN |
| Held-out complete-course drift, false closures, restart accuracy | BLOCKED on complete-loop/reference data collection |
| Successful autonomous full lap | UNRUN |

Package-test command after sourcing ROS, the venv and workspace installation:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOG_DIR=/tmp/cone_slam_ros_logs ROS_DOMAIN_ID=168 \
  python3 /usr/bin/colcon --log-base ../log test --base-paths src \
  --build-base ../build --install-base ../install \
  --packages-select cone_slam --event-handlers console_direct+
```

Machine-readable test output: workspace `build/cone_slam/pytest.xml`; existing
planner results: `build/path_planning/test_results/path_planning/`.
ROS integration cases use local in-process communication and injected sensor
messages. They do not establish cross-machine DDS delivery or hardware timing.
The installed-executable smoke was also run outside the sandbox on ROS domain 168
with TF publication disabled to verify startup without sandbox socket restrictions.

Reproduced failures fixed during development: adjacent cone suppression, duplicate
constraints from delayed captures, falsely available poses after solver failure,
non-expiring tentative cones during sensor silence, covariance mutation on map
load, and double ROS shutdown on SIGINT. Tests were run failing before each fix.

Two inherited build/environment issues also blocked reproducibility: #167's
`perception_observation/setup.py` listed two absent Markdown files, and the Makefile
invoked system-Python colcon despite activating the venv. Packaging now lists the
existing file, and Makefile invokes colcon with the activated interpreter. This
workspace's venv was changed to `include-system-site-packages = true`, matching
`scripts/setup.sh`'s intended ROS-compatible venv configuration. A failed initial
package-test attempt using system NumPy 1.21.5 with user SciPy 1.15.3 was superseded
by the passing venv-based run above. Disabling pytest plugin auto-loading avoids
an unrelated ROS launch_testing/pytest-version conflict for these tests.

## Recording and new-cone follow-up

See [REPLAY_VALIDATION.md](REPLAY_VALIDATION.md) for seven raw detector replays,
21 final sensor-ablation runs, actual planner integration, and explicit failed/
unrun acceptance criteria. Final direct pytest run: **23 passed** (18 core,
5 ROS cases, including unknown-color LiDAR adapter output). Planner unit tests:
**6 passed**, including straight, curved and ambiguous unknown-color geometry.
The installed planner's synthetic DDS integration also passed both turn directions
and empty-input clearing. Saved-map restart on the first recorded pass still failed
to acquire localization; full-loop and target-hardware acceptance remain open.
