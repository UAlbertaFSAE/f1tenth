# Perception

## What goes here?
The `perception/` folder should contain any ROS packages that are related to percieving the environment

## Packages

### Detection Camera
- Runs our YOLO model to detect cones from the ZED camera outputs

### Cone Transformer
- A node to transform the cone detections into the correct frames for processing

## Submodules
- This folder contains the our ZED camera wrapper along with the Livox SDK and ROS2 driver
