# Safety Node
Performs automatic emergency braking to avoid collisions while testing algorithms on the car

code and concepts from UWaterloo's [f1tenth repo](https://github.com/CL2-UWaterloo/f1tenth_ws/tree/main/src/safety_node)

### Running Instructions
To have safety node running properly, you need to launch the ZED ROS2 wrapper (necessary for laserscans). So before launching the safety node with `ros2 launch safety_node safety_node`, run `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i` in a separate terminal.
