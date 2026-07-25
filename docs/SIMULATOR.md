# Simulator Info

We use the [F1Tenth ROS-Gym bridge](https://github.com/f1tenth/f1tenth_gym_ros) simulation environment to test some algorithms for our RC car. This file contains information for using the simulator.

### Simulator Start Up

1. go to http://localhost:8080/vnc.html in your browser to see the NoVNC client,press connect

## Local Workflow (map_generator + cone_detector_sim)

This is the local (non-docker) setup: draw a track, export it to a cone CSV, point the sim at it, run.

### 1. Create a track

`map_generator` is a standalone GUI, no ROS needed to run it:

```bash
cd f1tenth_ws
source venv/bin/activate
cd src/src/simulation/map_generator
python3 main.py
```

- **Draw Mode**: click to add centerline points. The left/right track edges and cones update live as you draw.
- **Move Point** / **Delete Point**: switch mode, click a point to drag or remove it.
- Toolbar params: **Track Width** (default 3m), **Cone Spacing** (default 1m), **Spline Res**, **Closed track** checkbox for loops.
- **Undo** / **Redo** / **Clear** as needed.
- **NEXT ->** shows the cone preview (blue/yellow) with track stats (length, cone count, avg spacing).
- **Export CSV** writes `id,x,y,color` — this is the file the sim reads.
- **Open CSV** loads and previews an existing track CSV instead of drawing a new one.

### 2. Point the sim at that CSV and run

From the folder (`f1tenth_ws/src`):

```bash
make run_sim CSV=/path/to/your_track.csv
```

This runs `scripts/set_track.py` under the hood, which:
- computes the ego spawn pose from the CSV's first station,
- writes that `csv_path` into both `cone_detector_sim` and `map_generator`'s `track_map_publisher` configs,
- forces `f1tenth_gym_ros`'s map to `maps/blank` (the track itself is never rasterized into the map image — it's published separately as ground truth),

then rebuilds `f1tenth_gym_ros`, `cone_detector_sim`, `map_generator`, and launches the full stack (gym bridge, pure_pursuit, triangulator, cone_detector_sim, RViz).

Run `make src run_sim` with no `CSV=` to relaunch with whatever CSV/spawn pose is already set in the configs (skips the rebuild-and-resync step).

### What you'll see in RViz

- **Map** (`/map`) — always blank, just the occupancy grid nav2 expects.
- **TrackMap** (`/track_map`) — the whole track's ground truth (left/right boundary + centerline), latched from the CSV.
- **ConeViewMarkers** (`/cone_view_markers`) — the cones `cone_detector_sim` currently "sees" from the car's live pose (limited FOV/range, simulates a real cone detector).
- **ConeMarkers** (`/triangulation_markers`) — `path_planning`'s accumulated cones + triangulation + waypoints fed to `pure_pursuit`.

Both `cone_detector_sim` and `path_planning`(`triangulator`) have a `view_persist` config value (default `false`): false clears each node's markers every frame (only the current view shown); true keeps every frame's markers on screen (a visual trail) instead of replacing them.
