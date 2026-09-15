#!/usr/bin/env python3
# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""Cone detection from the ZED stream.

Runs YOLO over the rectified colour image, reads each detection's depth, projects it into
metric space with the camera intrinsics, transforms it into the configured fixed frame,
and publishes the result as rc_interfaces/Cones on ``/cone_positions``.

The transform happens here rather than in a downstream node because this is the node that
holds the detection, its frame and its timestamp together.
"""

import math
import os
import traceback

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)
import torch
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rc_interfaces.msg import Cone, Cones
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO


class ConePublisher(Node):
    """ROS 2 node for cone detection using YOLO and depth estimation."""

    def __init__(self) -> None:
        """Initialize the ConePublisher node with parameters and subscriptions."""
        super().__init__("camera_detection")

        # Declare parameters with defaults from YAML
        self.declare_parameter("depth_node", "/zed/zed_node")
        self.declare_parameter("camera_node", "/zed/zed_node")
        self.declare_parameter("publishing_topic", "/cone_positions")
        self.declare_parameter("model_file", "")
        self.declare_parameter("classes_file", "")
        self.declare_parameter("include_depth", True)
        self.declare_parameter("visualize", False)
        self.declare_parameter("detection_confidence", 0.5)
        self.declare_parameter("imgsz", 832)
        self.declare_parameter("publish_rate_hz", 30)
        self.declare_parameter("apply_tf", True)
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("tf_timeout_sec", 0.5)

        # Get parameters
        self.depth_node = self.get_parameter("depth_node").value
        self.camera_node = self.get_parameter("camera_node").value
        self.publishing_topic = self.get_parameter("publishing_topic").value
        self.model_file = self.get_parameter("model_file").value
        self.classes_file = self.get_parameter("classes_file").value
        self.include_depth = self.get_parameter("include_depth").value
        self.visualize = self.get_parameter("visualize").value
        self.detection_confidence = self.get_parameter("detection_confidence").value
        self.imgsz = self.get_parameter("imgsz").value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.apply_tf = self.get_parameter("apply_tf").value
        self.target_frame = self.get_parameter("target_frame").value
        self.tf_timeout_sec = self.get_parameter("tf_timeout_sec").value

        self.get_logger().info("Starting detection camera node")
        self.get_logger().info(f"  Model file: {self.model_file}")
        self.get_logger().info(f"  Classes file: {self.classes_file}")
        self.get_logger().info(f"  Publishing to: {self.publishing_topic}")
        self.get_logger().info(f"  Include depth: {self.include_depth}")
        self.get_logger().info(f"  Visualize: {self.visualize}")
        self.get_logger().info(f"  Confidence threshold: {self.detection_confidence}")
        self.get_logger().info(f"  Inference size: {self.imgsz}")
        self.get_logger().info(f"  Apply TF: {self.apply_tf}")
        self.get_logger().info(f"  Target frame: {self.target_frame}")

        # TF is only set up when it is actually going to be used; a Buffer and
        # TransformListener that nothing queries still costs a subscription to
        # /tf and /tf_static.
        self.tf_buffer: Buffer | None = None
        self.tf_listener: TransformListener | None = None
        self.tf_timeout = Duration(seconds=float(self.tf_timeout_sec))
        if self.apply_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info(
                f"TF enabled: cone positions will be published in '{self.target_frame}'"
            )
        else:
            self.get_logger().warn(
                "TF disabled: cone positions are published in the camera optical "
                "frame, not a fixed frame. Downstream planning expects a fixed frame."
            )

        # Detect available device (GPU/CPU)
        self.cuda_available = False
        self.device = "cpu"
        if torch is not None:
            try:
                if torch.cuda.is_available():
                    self.cuda_available = True
                    self.device = "cuda:0"
            except Exception:
                self.cuda_available = False
                self.device = "cpu"
        self.get_logger().info(
            f"Device selected: {self.device} (cuda_available={self.cuda_available})"
        )

        # Publishers / subscribers
        self.publisher_ = self.create_publisher(Cones, self.publishing_topic, 10)

        # Publishers for visualization
        if self.visualize:
            self.viz_image_pub = self.create_publisher(
                Image, "/detection_visualization/detections", 10
            )
            self.viz_depth_pub = self.create_publisher(
                Image, "/detection_visualization/depth", 10
            )
            self.get_logger().info("Visualization enabled - publishing to:")
            self.get_logger().info("  /detection_visualization/detections")
            self.get_logger().info("  /detection_visualization/depth")

        # Subscribe to left image (rectified color), depth registered to left, and camera_info for intrinsics
        self.left_image_sub = self.create_subscription(
            Image,
            f"{self.camera_node}/rgb/color/rect/image",
            self.left_image_callback,
            10,
        )

        if self.include_depth:
            self.depth_sub = self.create_subscription(
                Image,
                f"{self.depth_node}/depth/depth_registered",
                self.depth_callback,
                10,
            )

            self.caminfo_sub = self.create_subscription(
                CameraInfo,
                f"{self.camera_node}/depth/depth_registered/camera_info",
                self.caminfo_callback,
                10,
            )
            self.get_logger().info(
                f"Subscribed to depth at: {self.depth_node}/depth/depth_registered"
            )
            self.get_logger().info(
                f"Subscribed to camera info at: {self.camera_node}/depth/depth_registered/camera_info"
            )

        self.get_logger().info(
            f"Subscribed to image at: {self.camera_node}/rgb/color/rect/image"
        )

        # State
        self.bridge = CvBridge()
        self.model = self.load_model()
        self.classes = self.load_classes(self.classes_file)
        self.latest_depth: np.ndarray | None = None  # numpy array (meters)
        self.latest_depth_msg = None  # original ROS message for visualization
        self.latest_caminfo: CameraInfo | None = None  # sensor_msgs/CameraInfo
        # (fx, fy, cx, cy), parsed once per CameraInfo message
        self.intrinsics: tuple[float, float, float, float] | None = None
        self.get_logger().info("ConePublisher node started successfully.")

    # -------------------------
    # Callbacks to store sensor data
    # -------------------------
    def depth_callback(self, msg: Image) -> None:
        """Store latest depth image as a numpy array (in meters).

        Expecting depth image encoding to be 32FC1 or similar.
        """
        try:
            # depth_registered commonly published as 32FC1 (meters)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            # Convert to numpy float32 for safePoseWithCovariancer operations
            self.latest_depth = np.array(depth_image, dtype=np.float32)
            # Store original message for visualization
            self.latest_depth_msg = msg
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge depth conversion failed: {e}")
        except Exception:
            self.get_logger().error(
                "Unexpected error converting depth image:\n" + traceback.format_exc()
            )

    def caminfo_callback(self, msg: CameraInfo) -> None:
        """Store the camera intrinsics from the latest CameraInfo message.

        Parsed once here rather than per frame in the detection path.
        """
        self.latest_caminfo = msg
        try:
            k = msg.k if hasattr(msg, "k") else msg.K
            fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        except Exception:
            self.get_logger().warn(
                "Failed to read camera intrinsics from CameraInfo message."
            )
            self.intrinsics = None
            return

        if (
            not math.isfinite(fx)
            or not math.isfinite(fy)
            or not math.isfinite(cx)
            or not math.isfinite(cy)
            or fx == 0.0
            or fy == 0.0
        ):
            self.get_logger().warn(
                f"Unusable camera intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}"
            )
            self.intrinsics = None
            return

        self.intrinsics = (fx, fy, cx, cy)

    # -------------------------
    # Main image processing callback (YOLO runs here)
    # -------------------------
    def left_image_callback(self, msg: Image) -> None:
        """Process incoming left rectified image with YOLO detection.

        Runs YOLO detection, looks up depth in the registered left depth map,
        projects each detection to 3D camera coordinates, transforms it into
        the configured target frame, and publishes rc_interfaces/Cones.
        """
        # Convert ROS Image to cv2
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge image conversion failed: {e}")
            return
        except Exception:
            self.get_logger().error(
                "Unexpected error converting left image:\n" + traceback.format_exc()
            )
            return

        # Run YOLO (pass device to offload to GPU if available)
        try:
            results = self.model(
                cv_image,
                conf=self.detection_confidence,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )
        except Exception:
            self.get_logger().error("YOLO model run failed:\n" + traceback.format_exc())
            return

        # Build Cones
        cone_list, detections_info = self.process_detections(results, msg.header)
        cones_msg = Cones()
        cones_msg.cones = cone_list

        self.publisher_.publish(cones_msg)

        # Visualization
        if self.visualize:
            self.publish_visualization(cv_image, detections_info, msg.header)

    # -------------------------
    # Detection -> Object conversion
    # -------------------------
    def process_detections(self, results: list, source_header: Header) -> tuple:
        """Convert YOLO results into a list of rc_interfaces/Cone messages.

        Uses latest_depth and the parsed camera intrinsics to compute a 3D position for each
        detection, then transforms it into the configured target frame. A
        detection is dropped rather than published in the wrong place when its
        label is not a known cone color, when no valid depth is available, or
        when the TF lookup fails.

        Args:
            results: YOLO results for one frame.
            source_header: Header of the image the detections came from; carries
                the frame_id and stamp the TF lookup needs.

        Returns:
            tuple: (cones, detections_info) where detections_info is for visualization.
        """
        cones = []
        detections_info = []  # Store bbox, label, confidence, position for visualization

        # Bound to a local so it cannot be swapped out by the depth callback
        # part-way through this frame's detections.
        depth = self.latest_depth if self.include_depth else None
        if depth is None or self.intrinsics is None:
            # Throttled: this fires per frame while depth or CameraInfo is
            # missing, which is every frame until the ZED comes up.
            self.get_logger().warn(
                "No depth or intrinsics available; no cone positions can be computed.",
                throttle_duration_sec=5.0,
            )

        for result in results:
            for box in result.boxes:
                try:
                    # Extract bbox coordinates
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    confidence = float(box.conf[0].item())
                    class_id = int(box.cls[0].item())
                except Exception:
                    try:
                        x1, y1, x2, y2 = (float(v) for v in box.xyxy)
                        confidence = float(box.conf)
                        class_id = int(box.cls)
                    except Exception:
                        self.get_logger().warn(
                            "Unexpected box format from YOLO; skipping box."
                        )
                        continue

                class_label = (
                    self.classes[class_id]
                    if class_id < len(self.classes)
                    else f"unknown_{class_id}"
                )

                # A detection whose label is not a cone color cannot be placed on
                # a track boundary, so it is dropped here rather than published
                # with an empty color for a downstream node to filter out.
                color = self.normalize_color(class_label)
                if not color:
                    self.get_logger().debug(
                        f"Dropping detection with unrecognized label '{class_label}'"
                    )
                    continue

                position_3d = self.locate_cone(x1, y1, x2, y2, depth, source_header)

                # Recorded even when no position could be computed, so the
                # visualization still shows what the model saw.
                detections_info.append(
                    {
                        "bbox": (int(x1), int(y1), int(x2), int(y2)),
                        "label": class_label,
                        "confidence": confidence,
                        "position_3d": position_3d,
                    }
                )

                if position_3d is None:
                    continue

                x, y, _z = position_3d
                cone = Cone()
                cone.x = float(x)
                cone.y = float(y)
                cone.color = color
                cones.append(cone)

                self.get_logger().debug(
                    f"Cone '{color}' at x={x:.2f}, y={y:.2f} (conf={confidence:.2f})"
                )

        return cones, detections_info

    def locate_cone(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        depth: np.ndarray | None,
        source_header: Header,
    ) -> tuple[float, float, float] | None:
        """Compute a cone position from a bounding box and the depth map.

        Takes the median of the valid depth values inside the box (rather than
        the centre pixel, which is often invalid on a thin object) and projects
        the box centre through the camera intrinsics, then transforms the result
        into the target frame when TF is enabled.

        Args:
            x1: Left edge of the bounding box, in pixels.
            y1: Top edge of the bounding box, in pixels.
            x2: Right edge of the bounding box, in pixels.
            y2: Bottom edge of the bounding box, in pixels.
            depth: Latest depth image in meters, or None when unavailable.
            source_header: Header carrying the source frame_id and stamp.

        Returns:
            The (x, y, z) position, or None when it could not be determined.
        """
        if depth is None or self.intrinsics is None:
            return None

        fx, fy, cx, cy = self.intrinsics
        h, w = depth.shape[:2]

        # clamp bbox to image bounds
        x1_i = max(0, int(math.floor(x1)))
        y1_i = max(0, int(math.floor(y1)))
        x2_i = min(w - 1, int(math.ceil(x2)))
        y2_i = min(h - 1, int(math.ceil(y2)))

        if x2_i <= x1_i or y2_i <= y1_i:
            self.get_logger().warn(
                "Invalid bbox size after clamping; skipping detection."
            )
            return None

        # Median of valid (non-zero, non-nan) depth values in the box
        depth_roi = depth[y1_i : y2_i + 1, x1_i : x2_i + 1]
        if depth_roi.size == 0:
            return None

        valid_mask = np.isfinite(depth_roi) & (depth_roi > 0.001)  # ignore zeros
        if not np.any(valid_mask):
            return None

        z = float(np.median(depth_roi[valid_mask]))

        # Use bbox center for x,y projection, clamped to image bounds
        u = max(0, min(w - 1, int((x1 + x2) / 2.0)))
        v = max(0, min(h - 1, int((y1 + y2) / 2.0)))

        # Convert pixel + z -> camera coords
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        if not self.apply_tf:
            return (x, y, z)

        # Publishing an untransformed position on a topic documented as being in
        # target_frame would put the cone somewhere wrong, which is worse than
        # not publishing it at all.
        return self.transform_cone_position(x, y, z, source_header)

    def normalize_color(self, raw_label: str) -> str:
        """Map a model class label onto a canonical cone color.

        Args:
            raw_label: Class label as it appears in the classes file.

        Returns:
            str: One of "blue", "yellow", "orange", "large_orange", or an empty
                string when the label is not a cone color.
        """
        label = raw_label.lower()

        # Checked before plain "orange" so that "large_orange" is not swallowed
        # by the substring match below.
        if "large_orange" in label or "large orange" in label:
            return "large_orange"
        if "blue" in label:
            return "blue"
        if "yellow" in label:
            return "yellow"
        if "orange" in label:
            return "orange"
        return ""

    def transform_cone_position(
        self, cone_x: float, cone_y: float, cone_z: float, source_header: Header
    ) -> tuple[float, float, float] | None:
        """Transform a cone position from the camera frame into the target frame.

        Args:
            cone_x: X coordinate in the source frame.
            cone_y: Y coordinate in the source frame.
            cone_z: Z coordinate in the source frame.
            source_header: Header carrying the source frame_id and stamp.

        Returns:
            The transformed (x, y, z), or None when the lookup fails.
        """
        if self.tf_buffer is None:
            return None

        input_point = PointStamped()
        input_point.header = source_header
        input_point.point.x = float(cone_x)
        input_point.point.y = float(cone_y)
        input_point.point.z = float(cone_z)

        try:
            output_point = self.tf_buffer.transform(
                input_point,
                self.target_frame,
                timeout=self.tf_timeout,
            )
        except TransformException as ex:
            # Throttled: a missing transform fails for every cone in every
            # frame, and one line per cone drowns the log.
            self.get_logger().warn(
                f"TF transform to '{self.target_frame}' failed: {ex}",
                throttle_duration_sec=5.0,
            )
            return None

        return (
            float(output_point.point.x),
            float(output_point.point.y),
            float(output_point.point.z),
        )

    # -------------------------
    # Visualization
    # -------------------------
    def publish_visualization(
        self, cv_image: np.ndarray, detections_info: list, header: dict
    ) -> None:
        """Publish visualization images with bounding boxes and depth information.

        Creates side-by-side visualization when depth is available.
        """
        try:
            # Draw detections on the image
            viz_image = cv_image.copy()

            for det in detections_info:
                x1, y1, x2, y2 = det["bbox"]
                label = det["label"]
                confidence = det["confidence"]
                position_3d = det["position_3d"]

                # Draw bounding box
                color = (0, 255, 0)  # Green for cones
                cv2.rectangle(viz_image, (x1, y1), (x2, y2), color, 2)

                # Prepare label text
                label_text = f"{label}: {confidence:.2f}"
                if position_3d is not None:
                    x, y, z = position_3d
                    label_text += f" | D:{z:.2f}m"
                    # Add position text below bbox
                    pos_text = f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}"
                    cv2.putText(
                        viz_image,
                        pos_text,
                        (x1, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2,
                    )

                # Draw label above bbox
                (text_width, text_height), _ = cv2.getTextSize(
                    label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                )
                cv2.rectangle(
                    viz_image,
                    (x1, y1 - text_height - 10),
                    (x1 + text_width, y1),
                    color,
                    -1,
                )
                cv2.putText(
                    viz_image,
                    label_text,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 0),
                    2,
                )

            # Publish detection visualization
            try:
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding="bgr8")
                viz_msg.header = header
                self.viz_image_pub.publish(viz_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Failed to publish visualization image: {e}")

            # Publish depth visualble
            if self.include_depth and self.latest_depth is not None:
                try:
                    # Normalize depth for visualization (0-10m range)
                    depth_viz = self.latest_depth.copy()
                    depth_viz = np.nan_to_num(
                        depth_viz, nan=0.0, posinf=10.0, neginf=0.0
                    )
                    depth_viz = np.clip(depth_viz, 0.0, 10.0)
                    depth_viz_normalized = (depth_viz / 10.0 * 255).astype(np.uint8)

                    # Apply colormap
                    depth_colored = cv2.applyColorMap(
                        depth_viz_normalized, cv2.COLORMAP_JET
                    )

                    # Draw detection boxes on depth image too
                    for det in detections_info:
                        x1, y1, x2, y2 = det["bbox"]
                        cv2.rectangle(
                            depth_colored, (x1, y1), (x2, y2), (255, 255, 255), 2
                        )

                    # Publish depth visualization
                    depth_viz_msg = self.bridge.cv2_to_imgmsg(
                        depth_colored, encoding="bgr8"
                    )
                    depth_viz_msg.header = header
                    self.viz_depth_pub.publish(depth_viz_msg)
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to publish depth visualization: {e}"
                    )

        except Exception as e:
            self.get_logger().error(
                f"Visualization failed: {e}\n{traceback.format_exc()}"
            )

    # -------------------------
    # Model loading
    # -------------------------
    def load_model(self) -> YOLO:
        """Load YOLO model from the configured model file.

        Returns:
            YOLO: Loaded YOLO model instance.
        """
        try:
            # Check if model file exists
            if not os.path.exists(self.model_file):
                self.get_logger().error(f"Model file not found: {self.model_file}")
                raise FileNotFoundError(f"Model file not found: {self.model_file}")

            # Load model based on file extension
            if self.model_file.endswith(".onnx"):
                self.get_logger().info(f"Loading ONNX model from: {self.model_file}")
                model = YOLO(self.model_file, task="detect")
            elif self.model_file.endswith(".pt"):
                self.get_logger().info(f"Loading PyTorch model from: {self.model_file}")
                model = YOLO(self.model_file)
            else:
                self.get_logger().warn(
                    f"Unknown model format, attempting to load: {self.model_file}"
                )
                model = YOLO(self.model_file)

            self.get_logger().info("YOLO model loaded successfully.")
            return model
        except Exception as e:
            self.get_logger().error(
                f"Failed to load YOLO model: {e}\n{traceback.format_exc()}"
            )
            raise

    def load_classes(self, file_path: str) -> list:
        """Load class names from file.

        Args:
            file_path: Path to the classes file.

        Returns:
            list: List of class names.
        """
        try:
            if not os.path.exists(file_path):
                self.get_logger().warn(
                    f"Classes file not found: {file_path}, using default 'cone' label"
                )
                return ["cone"]

            with open(file_path) as f:
                classes = [line.strip() for line in f.readlines()]
            self.get_logger().info(f"Loaded {len(classes)} classes from {file_path}")
            return classes
        except Exception as e:
            self.get_logger().error(f"Failed to load classes file: {e}")
            return ["cone"]  # Default fallback


def main(args: list | None = None) -> None:
    """Main entry point for the ConePublisher node.

    Args:
        args: Command-line arguments.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = ConePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
