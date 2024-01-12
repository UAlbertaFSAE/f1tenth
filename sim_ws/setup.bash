echo "source /opt/ros/humble/setup.bash" > ~/.bashrc
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# TODO: setup tmux server for running the teleop keys and the simulator at the same time
ros2 launch f1tenth_gym_ros gym_bridge_launch.py