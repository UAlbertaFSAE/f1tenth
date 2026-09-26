# Setup

Getting from a fresh clone to a workspace that builds and runs. If you are new to the team, read [ONBOARDING.md](ONBOARDING.md) first -- it covers the Docker/devcontainer route, which is the recommended path on Windows and macOS. This document covers the native Linux route and is the reference for what the tooling actually does.

## Workspace layout

This repository is cloned *as* the colcon workspace's `src/` directory. Clone it into a directory named `src` inside a workspace root of your choosing:

```bash
git clone git@github.com:UAlbertaFSAE/f1tenth.git f1tenth_ws/src
cd f1tenth_ws/src
```

That gives you:

```
f1tenth_ws/          workspace root -- build/, install/, log/, venv/ land here
└── src/             this repository
    └── src/         the ROS packages
```

The doubled `src/` is deliberate: colcon wants a workspace root that is not the git tree, and the Makefile resolves that root from its own location rather than from your current directory, so `make`, `make -C src` and `make -f src/Makefile` all write to the same place.

There are no submodules. Everything arrives with `git clone`.

## Prerequisites

- **Ubuntu 22.04 (jammy)**. `make deps` installs ROS 2 Humble automatically only on jammy. On any other distribution it stops and asks you to install Humble yourself, then re-run.
- **ZED SDK**, only if you intend to build or run the ZED camera packages. Requires a dedicated NVIDIA GPU. Download from [stereolabs.com](https://www.stereolabs.com/en-ca/developers/release). `make build` skips those packages, so you do not need it to get a working workspace.

Everything else -- Eigen3, CDT, rosdep dependencies, the Python venv -- is installed for you.

## Install

```bash
make deps
```

This runs `scripts/setup.sh`, which:

1. installs ROS 2 Humble if it is missing (jammy only),
2. installs the apt prerequisites and initialises rosdep,
3. reports whether CUDA and the ZED SDK are present, so you know whether the ZED packages will build,
4. installs CDT,
5. creates the Python venv at `f1tenth_ws/venv` and installs the workspace's rosdep and pip dependencies into it, including `f1tenth_gym`.

It does not touch `~/.bashrc` and defines no shell aliases. Every Makefile recipe sources ROS and the venv itself, so there is nothing to remember to source first.

## Build

```bash
make build
```

`make help` lists every target. The ones you will use:

| Target | What it does |
| --- | --- |
| `make deps` | Full environment setup (above) |
| `make build` | colcon build, minus `PACKAGES_IGNORE` |
| `make build_all` | colcon build, everything |
| `make package <pkg> [<pkg>..]` | Build only the named packages |
| `make clean` | Remove `build/`, `install/` and `log/` |
| `make rebuild` | `clean` then `build` |
| `make test` | `colcon test` plus full results |
| `make lint` | Every linter CI runs, over all of `src/` |
| `make run_auto` | Launch the autonomous stack on the car |
| `make run_sim [CSV=<path>]` | Launch the simulator stack |

`make lint` is exactly what CI runs, so a green local lint is a green CI lint. Note that `make lint-cpp` needs `build/compile_commands.json` and will tell you to run `make build` first if it is missing.

### PACKAGES_IGNORE

A plain `make build` skips the vendored LiDAR and ZED packages:

```
livox_sdk2  livox_ros_driver2  zed_components  zed_ros2
```

They are slow to build and need SDKs that are not on every machine, and nothing in day-to-day development touches them. To build them:

```bash
make build_all              # everything
make package zed_wrapper    # just one
```

`PACKAGES_IGNORE` is a Makefile variable, so you can override it per invocation: `make build PACKAGES_IGNORE=`.

## Run

```bash
make run_auto                # the car
make run_sim                 # the simulator
```

Both are one launch file driven by a config profile:

```bash
ros2 launch launch_pkg fsae.launch.py                        # config.yaml
ros2 launch launch_pkg fsae.launch.py config:=sim_config.yaml
```

The profile in `src/common/launch_pkg/config/` decides which components come up -- ZED, LiDAR driver, camera detection, LiDAR filtering, path planning, pure pursuit, RViz, rosbag -- along with the ZED arguments, the static transforms and the recorded topics. Running without a piece of hardware is a config change, not a different launch file.

Manual driving is a separate bringup; see [MANUAL_DRIVING.md](MANUAL_DRIVING.md).

## Common problems

**The build runs out of memory.** colcon builds packages in parallel by default. Drop the worker count:

```bash
make build PARALLEL_WORKERS=1
```

**A ZED or LiDAR package fails to configure.** You are building packages whose SDK is not installed. Use `make build` rather than `make build_all`, or install the missing SDK.

**`asio` fails to build.**

```bash
sudo apt install -y ros-humble-asio-cmake-module
```

**Interface packages fail to configure, complaining about `catkin_pkg` or `em`.** The venv was created without `--system-site-packages` and can no longer see the Python packages ROS ships in `dist-packages`. Delete `f1tenth_ws/venv` and re-run `make deps`.

**`build/`, `install/` or `log/` show up inside the git tree.** You are running `colcon` by hand from inside the repository. Use the Makefile targets, which address every colcon path absolutely from the workspace root.
