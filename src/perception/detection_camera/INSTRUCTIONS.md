### Build the workspace
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
source install/setup.bash

### Run the ros2_zed node
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

### Run the cone detection model
In another terminal:
source install/setup.bash
ros2 launch detection_camera camera_detection.launch.py

### In another terminal
ros2 topic list
ros2 topic echo /cone_detections

### Troubleshooting
- If something is not working in the zed ROS2 wrapper, then check the config files in the `config` folder. One more possibililty is to check README.md file as it has somemore troubleshooting instructions.
