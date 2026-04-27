# FSAE F1Tenth stack
This is the instructions manual to get started and has solutions to some common bugs.

## File Structure
```text
.
├── common                  # Common packages used across the repo
│   ├── launch_pkg              # Main launch code
│   ├── rc_interfaces           # ROS messages
│   └── safety_node             # Ignored currently
├── hardware                # Hardware-level code (drivers)
│   └── f1tenth_system          # VESC / vehicle connection
├── navigation              # Vehicle navigation
│   ├── particle_filter         # SLAM / localization
│   ├── path_planning           # Waypoint generation
│   └── pure_pursuit            # Outputs steering + velocity
├── perception              # Vision / LiDAR perception
│   ├── camera_detection        # Camera-based cone detection
│   ├── livox_ros_driver2       # Livox Mid-360 ROS driver
│   ├── livox_SDK2              # Livox Mid-360 3rd party SDK
│   └── zed_wrapper             # ZED camera 3rd party SDK
└── simulation              # Simulation
    ├── detection_generator     # Simulates camera_detection from cone positions
    └── f1tenth_gym_ros         # Vehicle simulator
```

## Getting Started
Clone the repository into f1tenth/src

```bash
git clone https://github.com/UAlbertaFSAE/f1tenth.git f1tenth/src
cd f1tenth
```

Install the prerequisite packages:
1. Install [ZED SDK](https://www.stereolabs.com/en-ca/developers/release) (only if you want to run the zed_wrapper / camera on your pc). Else look at [Common Problems 1.1](#common-problems)
2. Eigen3: Can be installed using the apt repository (sudo apt install libeigen3-dev) or from the [Official website](https://libeigen.gitlab.io/news/eigen_3.4.1_released/)
3. [CDT](https://github.com/artem-ogre/CDT#installationbuilding)

```bash
sudo apt-get update --fix-missing
rosdep install -i --from-path src --rosdistro humble -y
```

Build the repository
```bash
colcon build --cmake-args -DDISTRO_ROS=${VERSION_HUMBLE} --parallel-workers 1 --symlink-install
```
## Run
```bash
source install/setup.bash
```
To run the autonomous code:
```bash
ros2 launch launch_pkg fsae.launch.py config:=src/src/common/launch_pkg/config/config.yaml
```

To run the simulator:
```bash
ros2 launch launch_pkg fsae.launch.py config:=src/src/common/launch_pkg/config/sim_config.yaml
```

## Common problems
1. If the colcon build is failing, it could be due to 1 of the 2 reasons below:
    1. Package is uninstalled.

    The repository depends on the ZED SDK, Livox SDK, and CDT. For the ZED SDK, a dedicated Nvidia GPU is required.

    Use the --packages-ignore in the colcon build to not build those packages
    ```bash
    colcon build --packages-ignore zed_components camera_detection livox_ros_driver2 livox_sdk2
    ```

    2. You are out of memory. If --parallel-workers is not mentioned by default ros will install all of the packages in parallel causing Out-of-memory errors and hence causes failures in building the packages. It is recommended that you use --parallel-workers 1 if this error occurs.

2. If asio failed:
``` bash
 sudo apt install -y ros-humble-asio-cmake-module
```