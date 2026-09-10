# Ualberta F1Tenth

Welcome to our autonomous remote control car repository. We are developing an autonomy stack for an [RC car](https://f1tenth.org/build.html) as a testing ground for algorithms and design decisions that may be included in our future Autonomous Electric vehicle. If you are just starting out, check out the [Contributing Guidelines](docs/CONTRIBUTING.md) document for more information on how to proceed with helping us develop our system!! Make sure to check out the resources section of the guideline if you are new to ROS or a new UAlberta formula team member as there is information about onboarding and learning resources.

**Important**: In general, the `docs/` folder contains lots of info pertaining to our stack, and this will be where many sources of information get put in the future. If ever you are lost, check there first!

**Note**: As of right now, we are only accepting contributions from UofA students.

## Layout

This repository is cloned *as* the colcon workspace's `src/` directory:

```
f1tenth_ws/          workspace root -- build/, install/, log/, venv/ land here
└── src/             this repository
    └── src/         the ROS packages
        ├── common/       launch_pkg, rc_interfaces
        ├── hardware/     f1tenth_system (vesc, ackermann_mux, teleop_tools, f1tenth_stack)
        ├── navigation/   particle_filter, path_planning, pure_pursuit
        ├── perception/   camera_detection, lidar_cone_filtering, livox_sdk2,
        │                 livox_ros_driver2, zed_wrapper
        └── simulation/   f1tenth_gym, f1tenth_gym_ros
```

There are no submodules. Everything arrives with `git clone`.

## Getting started

Everything goes through the `Makefile` at the root of this repo. Run `make help` for the
full list.

```bash
make deps        # ROS, system and rosdep dependencies, plus the Python venv
make build       # build, skipping the vendored livox and zed packages
make run_auto    # launch the autonomous stack on the car
make run_sim     # launch the simulator stack
```

`make build` skips the vendored LiDAR and ZED packages because they are slow and need SDKs
that are not on every machine. Use `make build_all` for everything, or
`make package zed_wrapper` for one of them.

`make lint` runs the same linters CI runs, over all of `src/`, so a green local lint is a
green CI run. `make test` runs `colcon test` and prints the results.

The autonomous stack is one launch file driven by a config profile:

```bash
ros2 launch launch_pkg fsae.launch.py                       # config.yaml, the car
ros2 launch launch_pkg fsae.launch.py config:=sim_config.yaml
```

The profile in `src/common/launch_pkg/config/` decides which components come up -- ZED,
LiDAR driver, camera detection, LiDAR filtering, path planning, pure pursuit, RViz, rosbag
-- along with the ZED arguments, the static transforms and the recorded topics. Running
without a piece of hardware is a config change, not a different launch file.

The hardware bringup stack is still its own launch:

```bash
ros2 launch f1tenth_stack bringup_launch.py
```
