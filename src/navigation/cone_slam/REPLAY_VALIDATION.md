# Recording validation — 2026-11-13

Branch: `168-KS/cone-slam`, based on `167-KS/sensor-messages` at
`50afd373f350a107062981df6d5f94b444e43869`. Working-tree implementation, not a
released commit. Artifacts are under workspace `replay_runs/20261113/`;
`final_source_sha256.json` fingerprints the evaluated source. Each raw report
includes bag, model and configuration SHA-256 hashes. Original bags were read-only.

## Coverage and method

All seven available bags were processed: 2,340 camera frames and 2,110 LiDAR
frames, using the production detector callbacks and SLAM capture-time processing.
These are short straight passes (approximately 6–11 m travel) plus a stationary
recording. There is no complete loop or surveyed landmark/pose ground truth.
Sparse detector annotations do not provide ground-truth SLAM accuracy.

Raw runs (`full/`) decode compressed RGB and recorded depth, run the existing
YOLO model and LiDAR geometric detector, and retain observation timestamps. Camera
callback dispatch is delayed 50 ms of recorded time to allow depth arrival.
SLAM workers are stepped synchronously in simulated time. The detector outputs,
odometry and transforms are cached in `observations.sqlite3`; recorded map-to-odom
transforms are excluded so SLAM owns its correction.

`verified/` reprocesses all seven caches with the final SLAM implementation, once
with both sensors, once LiDAR-only, and once camera-only (21 runs). This avoids
CPU inference delays contaminating the mapper's wall-clock availability checks.
Raw runs preceded the final constellation-matching fixes; use the final verified
results below for mapping behavior. The generated detector observations did not
change. No controller or hardware driver was started during tests.

| Bag | Camera frames | LiDAR frames | Peak confirmed, both | Pose available samples, both |
|---|---:|---:|---:|---:|
| 20261011_145034 | 432 | 378 | 22 | 273/361 |
| 20261011_145349 | 0 | 70 | 12 | 64/93 |
| 20261011_145438 | 265 | 252 | 12 | 183/253 |
| 20261024_122251 | 276 | 159 | 32 | 251/290 |
| 20261024_123317 | 290 | 291 | 30 | 361/396 |
| 20261024_123836 | 471 | 408 | 34 | 376/409 |
| 20261024_130513 | 606 | 552 | 42 | 438/521 |

All 21 final runs ended without a SLAM fault or non-finite graph. Eligible map
round trips preserved IDs, coordinates and covariance. Camera-only replay of the
stationary bag correctly produced no map because the bag contains no images.
Pose availability is a reported algorithm state, **not an accuracy measurement**.

Rejected inputs remain visible in reports: five startup batches across three bags
lacked capture-time TF/odometry; 22 camera positions across the last two bags were
invalid. Ten optimizer time-budget rejections occurred across the final both-sensor
runs. Budget failures withhold usable output rather than reporting success.
The reports include additional counts for individual-sensor runs. There were no
pending batches at the end. These results do not demonstrate continuous driving.

## Planner and new-cone takeover

The installed C++ planner was tested over isolated DDS with synthetic cone inputs:
measured-color straight, unknown-color straight, left/right 90-degree bends with
only the first two gates camera-labeled, the same bends entirely uncolored, and
empty-input path clearing. All passed. The synthetic bend radius is 2 m and track
width 0.92 m. Unit tests also check ambiguous-branch rejection. The ROS adapter
test verifies that LiDAR-only confirmed cones reach the planner while their map
colors remain unknown. These tests do not simulate detector errors or vehicle
dynamics and are not evidence of a real curved-track lap.

The real planner also consumes the first straight bag through SLAM using
`test/replay_planner.py`, at 0.5x playback with original timestamps. Machine-readable
final results are in `planner_final/report.json`. It checks finite odom-frame
paths and end-of-input clearing; it does not evaluate steering or track containment.
Final replay: 363 path messages, 104 nonempty, 259 empty, no SLAM fault, all finite
and in odom, final path empty. The pre-fallback baseline also had 104 nonempty
paths. Equal counts are a regression check, not proof of continuous driveability.

## Failed and unrun acceptance tests

**Recorded saved-map restart failed:** replaying the first bag from its saved map
with a changed odom origin `[20, -10, 1 rad]` never acquired localization.
`restart_offset_robust/` retains the failed result. The saved reference remained
unchanged. Synthetic offset relocalization passes, but that is insufficient to
claim recorded restart support is validated. Outlier-baseline and inactive-reference
regressions were reproduced and fixed; the recorded constellation still does not
meet the conservative acceptance criteria. Thresholds were not loosened to force
success.

Full-loop drift correction, held-out restart accuracy, false-closure rates, absolute
map error, ID-switch rates against truth, autonomous lap completion, and Jetson
25 W combined-workload performance remain unvalidated. Issue #168 must not be
closed based on these short recordings. Current hardware launch uses mapping mode,
not the failed saved-map-localization replay mode.

## Reproduction

Host: x86_64 Ubuntu/ROS Humble, CPU inference; Python 3.10, NumPy 1.26.4,
SciPy 1.15.3, torch 2.14.1+cpu, torchvision 0.29.2+cpu, ultralytics 8.4.212,
Open3D 0.20.0, OpenCV 4.11.0.86. This is not a Jetson runtime benchmark.
Full CPU detector replay takes longer than recording duration. Parameters and
callback timings are retained per run. Set BLAS/OpenMP thread counts to avoid
oversubscription; torch uses two threads after initial model warmup.

From the workspace root, source ROS, the venv, and `install/setup.bash`, then:

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
export ROS_LOG_DIR=/tmp/cone_replay_ros YOLO_CONFIG_DIR=/tmp/cone_slam_yolo
export ROS_DOMAIN_ID=176  # isolated from the vehicle
python src/src/navigation/cone_slam/test/replay_recordings.py \
  recordings/20261011_145034/rosbag --output /tmp/raw-replay
python src/src/navigation/cone_slam/test/replay_recordings.py \
  recordings/20261011_145034/rosbag --cache /tmp/raw-replay/observations.sqlite3 \
  --sensors lidar --output /tmp/lidar-only
python src/src/navigation/cone_slam/test/replay_planner.py \
  /tmp/raw-replay/observations.sqlite3 --output /tmp/planner-replay --rate 0.5
python src/src/navigation/cone_slam/test/check_lidar_planner.py
```

ROS/DDS tests require local socket access; sandbox socket restrictions can block
communication. No test launches `make run_auto`, the controller, or motor drivers.
