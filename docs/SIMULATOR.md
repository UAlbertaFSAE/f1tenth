# Simulator

The simulator runs our stack, not its own. It is the same launch file, the same node names and the same topics the car uses, with a config profile that turns the hardware off and the simulated sensors on. A parameter tuned in sim is the parameter the car uses, and integration bugs — topic name drift, QoS mismatches, TF frame errors — show up here instead of on the track.

There is no separate simulator image, container or checkout. `f1tenth_gym` and `f1tenth_gym_ros` are packages in this workspace and build with everything else.

## Run it

```bash
make build
make run_sim
```

That is `ros2 launch launch_pkg fsae.launch.py config:=sim_config.yaml`. To drive a different track:

```bash
make run_sim CSV=src/simulation/cone_detector_sim/data/eight.csv
```

Four tracks ship in `src/simulation/cone_detector_sim/data/`:

| Track | Stations | Shape |
| --- | --- | --- |
| `straight.csv` | 100 | straight line, the default |
| `eight.csv` | 93 | figure eight, crossing itself |
| `track-1.csv` | 200 | closed circuit |
| `track-2.csv` | 210 | closed circuit, tighter |

A track CSV is `id,x,y,color`: one row per cone, two rows per `id` — one `blue` and one `yellow` — forming a gate across the track. The centerline is reconstructed from the gate midpoints.

## What comes up

`sim_config.yaml` enables these on top of the usual stack:

| Component | Role |
| --- | --- |
| `simulator` | `f1tenth_gym_ros` gym bridge: physics, `/scan`, `/ego_racecar/odom`, TF, `/drive` |
| `cone_detector_sim` | replays the track CSV as `rc_interfaces/msg/Cones` on `/cone_positions`, gated by FOV and range |
| `track_map_publisher` | publishes the whole track as a latched `MarkerArray` on `/track_map` for RViz |

`camera_detection` is off — `cone_detector_sim` stands in for it, publishing the same message type on the same topic, so `path_planning` and `pure_pursuit` cannot tell the difference. `pure_pursuit` runs unmodified against the sim's odometry, which is what makes lookahead, speed profile and gain tuning a loop of seconds rather than a booking of track time.

The occupancy grid is always `maps/blank`. The track is never rasterised into the map image; it exists as the cone CSV, published as ground truth by `track_map_publisher` and served frame by frame to the stack by `cone_detector_sim`.

## How CSV= works

`make run_sim CSV=<path>` runs `scripts/set_track.py`, which writes three things that have to agree:

- `csv_path` in `cone_detector_sim/config/config.yaml`
- `csv_path` in `map_generator/config/config.yaml`
- `sx`, `sy` and `stheta` in `f1tenth_gym_ros/config/sim.yaml`

The spawn pose is computed from the CSV: the car starts at the midpoint of the first gate, facing the next one. Pass a station index as `STATION=<n>` to start somewhere else on the track.

```bash
make run_sim CSV=src/simulation/cone_detector_sim/data/track-1.csv STATION=40
```

Those configs are rewritten in the source tree, so `make run_sim CSV=...` rebuilds the three affected packages before launching. Running `make run_sim` afterwards with no `CSV=` keeps whatever track was last set.

Leaving `csv_path` empty resolves to the bundled `straight.csv` from the package share, so a fresh clone runs without setting a track first.

## Authoring a track

`map_generator` is the track-authoring tool. It draws a track from a spline, lays cones along both boundaries and writes the CSV:

```bash
ros2 run map_generator map_generator_gui
```

Save the result somewhere under `data/`, then point the sim at it with `make run_sim CSV=<path>`. The CSV is the single source of truth from that point on — one file drives the cones, the ground-truth markers and the spawn pose.

`src/simulation/cone_detector_sim/scripts/generate_tracks.py` generates track CSVs without the GUI, for scripted or batch cases.

## Topics

| Topic | Type | From |
| --- | --- | --- |
| `/scan` | `LaserScan` | gym bridge |
| `/ego_racecar/odom` | `Odometry` | gym bridge |
| `/drive` | `AckermannDriveStamped` | `pure_pursuit`, into the gym bridge |
| `/cone_positions` | `rc_interfaces/msg/Cones` | `cone_detector_sim` |
| `/cone_view_markers` | `MarkerArray` | `cone_detector_sim`, currently visible cones |
| `/track_map` | `MarkerArray` | `track_map_publisher`, whole track, latched |
