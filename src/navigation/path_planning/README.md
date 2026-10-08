# path_planning

ROS 2 C++17 package that generates a local waypoint path from cone detections and odometry. It uses the CDT library for Delaunay triangulation and a header-only B-spline implementation for smoothing.

The node and executable are named `path_planner`; the C++ class is `PathPlanner`.

## Planning pipeline

1. Store the latest odometry and smooth its heading.
2. Normalize cone colors, reject invalid coordinates, and remove same-side detections within the position tolerance.
3. Triangulate the cones and keep opposite-color edges within the configured length limit whose midpoints are ahead of the car.
4. Sort the gate midpoints by their projection onto the car's heading.
5. Smooth the ordered midpoints with a clamped B-spline and resample them into waypoints.
6. Publish the path and persistent visualization markers.

Planning runs inside the cone callback, without separate raw and processed buffers. Missing or stale odometry, stale/future cone observations, a mismatched cone frame, insufficient usable gates, or a planning error results in an empty path. Heading-based sorting is a simple ordering method and can be unsuitable for sharp turns or paths that double back.

## Topics and configuration

Default topics are configured in `config/config.yaml`:

| Direction | Topic | Message |
| --- | --- | --- |
| Input | `/cone_positions` | `rc_interfaces/msg/Cones` |
| Input | `/odom` | `nav_msgs/msg/Odometry` |
| Output | `/waypoints` | `nav_msgs/msg/Path` |
| Output | `/triangulation_markers` | `visualization_msgs/msg/MarkerArray` |

Cone coordinates and `header.frame_id` must already use the configured `frame_id`; the node does not transform them. The odometry frame must also match `frame_id`. Camera detections carry the original image capture timestamp in `header.stamp`, even after inference and coordinate transformation. Simulated detections carry their source odometry timestamp. The planner checks detection age but still uses the latest odometry, without historical pose matching.

Main settings include `position_tolerance`, `heading_smoothing_alpha`, `max_odom_age_s`, `max_cone_age_s`, `delaunay_max_edge_m`, and `waypoint_spacing_m`. `max_cone_age_s` defaults to 0.2 seconds. Position tolerance is checked independently on each axis, using `max(|dx|, |dy|)`. Clustering, track-width, gate, stopping, and extrapolation settings are currently read but do not affect the planning pipeline.

Recordings made with the previous headerless `Cones` message need message conversion before replay with this version.

## Build and run

From the workspace root, with the ROS environment sourced and CDT installed:

```bash
make -C src package path_planning
source install/setup.bash
ros2 launch path_planning path_planner.launch.py
```

The launch file accepts `config_file`, `odom_topic`, and `frame_id` overrides. For example:

```bash
ros2 launch path_planning path_planner.launch.py odom_topic:=/ego/odom frame_id:=map
```

## Possible performance improvements

These are candidates to measure, rather than verified speedups:

- **Profile first:** measure preprocessing, triangulation, smoothing, and publishing separately with representative cone counts and paths.
- **Reduce duplicate-search work:** preprocessing scans previously accepted cones for each detection, giving quadratic worst-case work. A spatial grid could limit comparisons to nearby cells while preserving the same-side and per-axis tolerance checks. Handle zero tolerance separately.
- **Batch edge calculations:** distances, midpoints, and heading projections could use SIMD or a vector library. The current point structures and indexed edge access may require data rearrangement; benchmark that overhead before introducing matrices.
- **Reuse storage:** reuse temporary vectors where practical and reserve marker point capacity to reduce allocations.
- **Cache sorting projections:** calculate each midpoint's forward projection once instead of recalculating it during every sort comparison. Benchmark the extra storage against the current small inputs.
- **Tune spline sampling:** measure whether fewer samples provide adequate path quality. De Boor's blending and distance-based resampling have sequential dependencies, so replacing their small loops with matrices is unlikely to help without further restructuring.
- **Control visualization cost:** every frame's markers are retained. Long runs can increase RViz memory and rendering cost; publishing markers less frequently could reduce growth while retaining each published frame.

Replacing a loop with `std::transform` alone does not guarantee vectorization or faster execution. Keep the simple loops unless measurements justify a change.

## License

MIT License — see [LICENSE](LICENSE).

Copyright (c) 2026 UAlbertaFSAE
