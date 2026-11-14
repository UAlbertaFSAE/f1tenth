# cone_slam

Standalone ROS 2 cone-landmark GraphSLAM for issue #168. Depends on #167's
`ConeObservations`, not on a planner, driving path, classifier change or controller.
The deployment target is Jetson Orin Nano, 25 W. Full-track driving is the system
objective; this package does not establish that the existing planner can select the
correct corridor through hairpins or parallel sections (#170).

## Build and run

From the workspace root:

```bash
make -C src package rc_interfaces perception_observation cone_slam path_planning launch_pkg
source /opt/ros/humble/setup.bash
source venv/bin/activate
source install/setup.bash
# Standalone: requires odometry and observation publishers, but no planner.
ros2 launch cone_slam cone_slam.launch.py output_map_path:=/tmp/course.json
# Integrated hardware profile: camera, LiDAR, SLAM, planner and driving stack.
make -C src run_auto
```

Do not run those two launches simultaneously. Only SLAM may publish `map -> odom`.
The integrated profile removes the configured static transform and rejects ZED
map-TF publication. The original `config.yaml` keeps SLAM disabled. Standalone
launch users must disable other map-TF owners themselves. `odom -> base_link`
continues to come from the existing odometry system. Frames are deliberately fixed
to `map`, `odom`, `base_link`; input mismatches are rejected rather than relabeled.

To include the actual source revision in a map, supply `source_commit` at launch.
Also retain the working-tree diff alongside validation results if testing uncommitted
changes. Camera detection is enabled in `slam_config.yaml`, using existing model
selection. Check calibration and camera availability before driving.

```bash
ros2 service call /cone_slam/save_map std_srvs/srv/Trigger '{}'
# Stop the first mapper before starting localization.
ros2 launch cone_slam cone_slam.launch.py mode:=localization map_path:=/tmp/course.json
# Extend an existing reference into a separate map file.
ros2 launch cone_slam cone_slam.launch.py mode:=update map_path:=/tmp/course.json output_map_path:=/tmp/course-updated.json
ros2 service call /cone_slam/reset std_srvs/srv/Trigger '{}'
```

Saving is queued: the service acknowledges admission, and `/slam/status.reason`
reports `map_saved` or a latched worker error. No automatic shutdown save overwrites
an existing map. Localization is read-only; update fixes reference geometry while
adding newly observed landmarks and accumulating labels. It does not silently
move a previously surveyed reference. Reset starts a new mapping ID, or reloads
and relocalizes the reference. It clears queued observations and odometry. New
odometry must arrive after reset. A clock rewind or odometry discontinuity requires
this explicit reset; it is never connected into the old graph as vehicle motion.

## Contracts

| Topic | Type | Meaning |
|---|---|---|
| `/perception/camera/observations` | `rc_interfaces/ConeObservations` | Independent capture-time camera evidence |
| `/perception/lidar/observations` | `rc_interfaces/ConeObservations` | Independent capture-time LiDAR evidence |
| `/odom` | `nav_msgs/Odometry` | Continuous odom/base_link pose, never a SLAM output |
| `/slam/map` | `rc_interfaces/ConeMap` | Persistent IDs, lifecycle, position covariance, existence and color evidence; transient-local |
| `/slam/pose` | `nav_msgs/Odometry` | Corrected vehicle pose in map; published only while available |
| `/slam/status` | `rc_interfaces/SlamStatus` | Map availability, independent pose availability/quality, revision, diagnostics |
| `/slam/planner_cones` | `rc_interfaces/Cones` | Confirmed local landmarks transformed into odom, including unknown colors; empty when unavailable |

The map/status publishers run at 10 Hz even without a path or planner. Map revision
changes on mapper updates, not on planner state. Original odometry timestamps are
preserved on pose/TF; status and local-map stamps describe the current snapshot.
An empty local map tells the planner there is no usable input. The integrated
profile sets `hold_timeout_s=0` so an old path cannot be held after unavailable
SLAM input. Global map corrections are transformed back into odom for the planner,
keeping pure pursuit's existing odom contract. No path data enters or is saved by
SLAM. Unknown-color landmarks stay useful to SLAM but cannot be invented into blue
or yellow boundaries by the adapter.

## Estimation and evidence

* Original sensor coordinates use TF at capture time to become body-relative XY
  measurements. The common reference is the ground projection of the cone axis.
  Present detectors approximate this with depth/surface centroids; the 8 cm
  calibration floor is a provisional allowance for viewpoint bias, **not a
  calibrated guarantee**. Actual sensor transforms, original positions/covariance,
  capture timestamps, camera projection and calibration hashes accompany saved factors.
* Odometry is buffered for 10 seconds (at most 4000 samples), interpolated only
  inside brackets separated by at most 0.2 s. Missing TF/odom batches wait at most
  0.5 s in bounded queues. Duplicate IDs include publisher session and sensor;
  data over one second behind the latest processed batch is discarded.
* Unknown measurement covariance uses `(0.20 + 0.01 * range_m)^2` per XY axis,
  plus the calibration floor. Invalid known covariance is rejected. All-zero
  odometry covariance uses a documented 5 cm / 0.03 rad fallback. Relative pose
  factors use a conservative covariance sum and distance-dependent process noise;
  they never treat successive absolute odometry estimates as independent precise
  ground truth or subtract correlated covariance matrices.
* Mutual unambiguous Mahalanobis association is one-to-one. Color does not override
  geometry. Nearby ambiguous/unmatched evidence is withheld. Distinct nearby
  detections within one capture can create separate IDs; exact/near-exact duplicates
  within 15 cm cannot. IDs are never recycled. There is no proximity-only merge of
  established landmarks; safe ambiguity is preferred over unsupported deduplication.
* Tentative landmarks require three independent keyframes to confirm and expire
  after three seconds without support. Confirmed landmarks become inactive after
  15 seconds but are not deleted for misses. Retired tentative records are removed
  to bound memory. Five **qualified** misses can mark a landmark suspect. The
  current ROS inputs do not provide a visibility/occlusion/health contract, so the
  node supplies **no negative evidence**. Persistent geometric clutter can still
  confirm; that must be measured with the actual detectors.
* One position factor per landmark/keyframe across both sensors avoids inflation
  from raw/fused or shared-depth observations. Camera can add measured color to a
  LiDAR position; image-only boxes label only a unique projected landmark, never
  create geometry. Color needs two votes and 75% agreement; contradictory evidence
  remains unknown. Color evidence and existence scores are not calibrated probabilities.
* SE(2) keyframes share landmark factors. Mapping fixes the first pose exactly;
  localization fixes reference landmarks and estimates all new poses. Sparse
  robust least squares warm-starts the growing graph, with joint marginal
  covariance solves for the current pose and up to 32 recent landmarks. This is
  bounded periodic graph reoptimization, **not an iSAM2 implementation**.
  Older covariance blocks retain their previous estimates. Shared keyframe pose
  uncertainty is modeled jointly; residual extrinsic correlation is only handled
  conservatively through the noise floor and evidence thinning, not a calibrated
  sensor-bias estimator.
* Loop/restart matching requires at least four spatially distributed landmarks,
  at least 75% constellation support, a unique rigid placement and three consistent
  captures with vehicle motion. Collinear and repeated placements are rejected.
  A search that exhausts its 80 ms budget cannot establish uniqueness and is rejected.
  Loop correction is limited to 3 m and 0.6 rad relative to odometry; restart has
  no old-odom-origin assumption. Evidence beyond these bounds reports unavailable
  instead of forcing a closure. Conservative gates can miss valid loops.
* Graph growth is capped at 600 keyframes, 1000 landmarks and 12000 factors.
  Capacity exhaustion reports degraded/unavailable status; it does not delete
  loop history to appear bounded. Repeated laps beyond these limits need a new
  localization session or a future validated marginalization strategy. Solver
  calls have a 250 ms admission/iteration budget and transactional commits. A
  single numerical kernel may finish beyond that budget; this is not hard real-time.
  ROS ingestion and publication remain separate from the worker.

Maps use schema 1 JSON, finite validated values, calibration compatibility checks,
atomic replacement, a 32 MiB file limit and persistent map/landmark IDs. Source
factor history is saved for audit/reprocessing; restart builds a fresh trajectory
against the saved landmark reference. No prior odom origin or driving path is reused.

See `VALIDATION.md` for provisional targets, actual checks and tomorrow's runbook.
Algorithm API reference: [SciPy sparse least_squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html).
