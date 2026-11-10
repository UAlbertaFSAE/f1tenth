#!/bin/bash
# Quick build and launch script for camera_detection package

clear
colcon build --packages-select camera_detection --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
source install/setup.bash

# Codes to run
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
ros2 launch camera_detection camera_detection.launch.py
# ros2 launch camera_detection camera_detection.launch.py parameter:=<value>
