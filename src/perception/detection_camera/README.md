# Detection Camera Package

A ROS 2 package for real-time cone detection using YOLO on ZED camera images with 3D position estimation.

## Features

- Real-time object detection using YOLO (supports both .pt and .onnx models)
- 3D position estimation using depth information from ZED camera
- Configurable detection parameters via YAML config file
- Optional visualization with RViz2 showing:
  - Detection bounding boxes with labels and confidence scores
  - 3D position information (X, Y, Z coordinates)
  - Side-by-side depth visualization
- Publishes detections as `zed_msgs/ObjectsStamped` messages

## Dependencies

- ROS 2 (Humble or later)
- ZED ROS 2 wrapper (`zed-ros2-wrapper`)
- Python packages:
  - `ultralytics` (YOLO)
  - `opencv-python` (cv2)
  - `numpy`
  - `cv_bridge`

## Installation

1. Clone this package into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
# Already included in zed-ros2-wrapper
```

2. Install Python dependencies:
```bash
pip install ultralytics opencv-python numpy
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select detection_camera
source install/setup.bash
```

## Configuration

Edit the configuration file at `config/camera_detection.yaml`:

```yaml
detection_camera:
  ros__parameters:
    # Node names
    depth_node: "/zed/zed_node"
    camera_node: "/zed/zed_node"
    publishing_topic: "/cone_positions"
    
    # Model paths
    model_file: "/home/nvidia/models/detection_model.onnx"
    classes_file: "/home/nvidia/models/classes.txt"
    
    # Features
    include_depth: true      # Enable 3D position estimation
    visualize: true          # Launch RViz2 visualization
    
    # Detection parameters
    detection_confidence: 0.5
    publish_rate_hz: 30
```

## Usage

### Launch with default parameters:
```bash
ros2 launch detection_camera camera_detection.launch.py
```

### Launch with custom parameters:
```bash
ros2 launch detection_camera camera_detection.launch.py \
  model_file:=/path/to/model.onnx \
  visualize:=true \
  detection_confidence:=0.6
```

### Launch without visualization:
```bash
ros2 launch detection_camera camera_detection.launch.py visualize:=false
```

### Using config file:
```bash
ros2 launch detection_camera camera_detection.launch.py \
  --params-file config/camera_detection.yaml
```

## Topics

### Subscribed Topics:
- `/zed/zed_node/left/image_rect_color` (sensor_msgs/Image) - RGB camera image
- `/zed/zed_node/depth/depth_registered` (sensor_msgs/Image) - Depth image (if `include_depth: true`)
- `/zed/zed_node/left/camera_info` (sensor_msgs/CameraInfo) - Camera intrinsics

### Published Topics:
- `/cone_positions` (zed_msgs/ObjectsStamped) - Detected objects with 3D positions
- `/detection_visualization/detections` (sensor_msgs/Image) - Image with bounding boxes (if `visualize: true`)
- `/detection_visualization/depth` (sensor_msgs/Image) - Colorized depth map with boxes (if `visualize: true`)

## Visualization

When `visualize: true`, the node publishes visualization images and automatically launches RViz2 with:
- Detection image showing:
  - Bounding boxes around detected cones
  - Confidence scores
  - 3D position coordinates (X, Y, Z in meters)
- Depth visualization showing:
  - Colorized depth map (0-10m range)
  - Detection bounding boxes overlaid

You can view these topics in RViz2 or use:
```bash
# View detection visualization
ros2 run rqt_image_view rqt_image_view /detection_visualization/detections

# View depth visualization
ros2 run rqt_image_view rqt_image_view /detection_visualization/depth
```

## Model Format

The package supports both:
- **PyTorch models** (.pt) - Standard YOLO format
- **ONNX models** (.onnx) - Optimized for inference

Ensure your `classes.txt` file has one class name per line, matching your model's class indices.

## Troubleshooting

### Model not found:
```
ERROR: Model file not found: /path/to/model.onnx
```
Solution: Update `model_file` path in config file or launch arguments.

### No depth information:
Check that:
- `include_depth: true` in config
- ZED node is publishing depth topics
- Depth topic name matches configuration

### No detections appearing:
- Lower `detection_confidence` threshold
- Verify model is trained for your objects
- Check camera image quality and lighting

### RViz not showing images:
- Verify topics are being published: `ros2 topic list`
- Check topic names in RViz match published topics
- Ensure `visualize: true` in configuration

## Performance Tips

1. Use ONNX model for faster inference
2. Adjust `publish_rate_hz` based on your needs
3. Disable visualization in production (`visualize: false`)
4. Use GPU acceleration if available (CUDA-enabled YOLO)

## License

Apache-2.0
