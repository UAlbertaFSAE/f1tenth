#!/bin/bash
# Quick build and launch script for detection_camera package

clear
colcon build --packages-select detection_camera --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
source install/setup.bash

# Codes to run
ros2 launch detection_camera camera_detection.launch.py
# ros2 launch detection_camera camera_detection.launch.py parameter:=<value>
