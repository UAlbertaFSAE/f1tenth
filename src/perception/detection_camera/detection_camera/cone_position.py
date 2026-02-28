#!/usr/bin/env python3
import math
import os
import traceback

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from zed_msgs.msg import BoundingBox2Di, Object, ObjectsStamped


class ConePublisher(Node):
    """ROS 2 node for cone detection using YOLO and depth estimation."""

    def __init__(self) -> None:
        """Initialize the ConePublisher node with parameters and subscriptions."""
        super().__init__("detection_camera")

        # Declare parameters with defaults from YAML
        self.declare_parameter("depth_node", "/zed/zed_node")
        self.declare_parameter("camera_node", "/zed/zed_node")
        self.declare_parameter("publishing_topic", "/cone_positions")
        self.declare_parameter("model_file", "")
        self.declare_parameter("classes_file", "")
        self.declare_parameter("include_depth", True)
        self.declare_parameter("visualize", False)
        self.declare_parameter("detection_confidence", 0.5)
        self.declare_parameter("publish_rate_hz", 30)

        # Get parameters
        self.depth_node = self.get_parameter("depth_node").value
        self.camera_node = self.get_parameter("camera_node").value
        self.publishing_topic = self.get_parameter("publishing_topic").value
        self.model_file = self.get_parameter("model_file").value
        self.classes_file = self.get_parameter("classes_file").value
        self.include_depth = self.get_parameter("include_depth").value
        self.visualize = self.get_parameter("visualize").value
        self.detection_confidence = self.get_parameter("detection_confidence").value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value

        self.get_logger().info("Starting detection camera node")
        self.get_logger().info(f"  Model file: {self.model_file}")
        self.get_logger().info(f"  Classes file: {self.classes_file}")
        self.get_logger().info(f"  Publishing to: {self.publishing_topic}")
        self.get_logger().info(f"  Include depth: {self.include_depth}")
        self.get_logger().info(f"  Visualize: {self.visualize}")
        self.get_logger().info(f"  Confidence threshold: {self.detection_confidence}")

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
        self.publisher_ = self.create_publisher(
            ObjectsStamped, self.publishing_topic, 10
        )

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
        """Store the latest CameraInfo (intrinsics).

        We'll use msg.k (or msg.K).
        """
        self.latest_caminfo = msg

    # -------------------------
    # Main image processing callback (YOLO runs here)
    # -------------------------
    def left_image_callback(self, msg: Image) -> None:
        """Process incoming left rectified image with YOLO detection.

        Runs YOLO detection, converts detections into zed_msgs/Object entries,
        looks up depth in the registered left depth map, projects to 3D coordinates,
        and publishes ObjectsStamped.
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
                device=self.device,
                verbose=False,
            )
        except Exception:
            self.get_logger().error("YOLO model run failed:\n" + traceback.format_exc())
            return

        # Build ObjectsStamped
        objects_list, detections_info = self.process_detections(results)
        objects_msg = ObjectsStamped()
        objects_msg.header = msg.header
        objects_msg.objects = objects_list

        self.publisher_.publish(objects_msg)

        # Visualization
        if self.visualize:
            self.publish_visualization(cv_image, detections_info, msg.header)

    # -------------------------
    # Detection -> Object conversion
    # -------------------------
    def process_detections(self, results: list) -> tuple:
        """Convert YOLO results into a list of zed_msgs/Object messages.

        Use latest_depth and latest_caminfo (if available) to compute 3D position.

        Returns:
            tuple: (cone_objects, detections_info) where detections_info is for visualization.
        """
        cone_objects = []
        detections_info = []  # Store bbox, label, confidence, position for visualization

        # Get intrinsics if possible
        fx = fy = cx = cy = None
        if self.include_depth and self.latest_caminfo is not None:
            try:
                k = (
                    self.latest_caminfo.k
                    if hasattr(self.latest_caminfo, "k")
                    else self.latest_caminfo.K
                )
                fx = float(k[0])
                fy = float(k[4])
                cx = float(k[2])
                cy = float(k[5])
            except Exception:
                self.get_logger().warn(
                    "Failed to read camera intrinsics from CameraInfo message."
                )
                fx = fy = cx = cy = None

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

                obj = Object()

                # Label fields
                class_label = (
                    self.classes[class_id]
                    if class_id < len(self.classes)
                    else f"unknown_{class_id}"
                )
                obj.label = class_label
                obj.label_id = class_id
                obj.sublabel = ""
                obj.confidence = confidence

                try:
                    bbox_2d = BoundingBox2Di()
                    try:
                        x1i = int(x1)
                        y1i = int(y1)
                        x2i = int(x2)
                        y2i = int(y2)

                        # corners layout (as documented):
                        # 0 ------- 1
                        # |         |
                        # |         |
                        # |         |
                        # 3 ------- 2
                        # Set all four corners using Keypoint2Di.kp (uint32[2])
                        bbox_2d.corners[0].kp[0] = x1i
                        bbox_2d.corners[0].kp[1] = y1i

                        bbox_2d.corners[1].kp[0] = x2i
                        bbox_2d.corners[1].kp[1] = y1i

                        bbox_2d.corners[2].kp[0] = x2i
                        bbox_2d.corners[2].kp[1] = y2i

                        bbox_2d.corners[3].kp[0] = x1i
                        bbox_2d.corners[3].kp[1] = y2i
                    except Exception as e:
                        self.get_logger().warn(f"Failed to set bbox corners: {e}")
                    obj.bounding_box_2d = bbox_2d
                except Exception:
                    self.get_logger().warn(
                        "Could not populate BoundingBox2Di exactly; continuing with minimal bbox."
                    )

                # Default values
                obj.position = [0.0, 0.0, 0.0]
                obj.position_covariance = [0.0] * 6
                obj.velocity = [0.0, 0.0, 0.0]
                obj.tracking_available = False
                obj.tracking_state = 0
                obj.action_state = 0

                position_3d = None  # for visualization

                # Try to compute 3D position using depth map and intrinsics (only if depth available)
                if (
                    self.include_depth
                    and self.latest_depth is not None
                    and fx is not None
                    and fy is not None
                    and cx is not None
                    and cy is not None
                    and not math.isnan(fx)
                    and not math.isnan(fy)
                ):
                    h, w = self.latest_depth.shape[:2]

                    # clamp bbox to image bounds
                    x1_i = max(0, int(math.floor(x1)))
                    y1_i = max(0, int(math.floor(y1)))
                    x2_i = min(w - 1, int(math.ceil(x2)))
                    y2_i = min(h - 1, int(math.ceil(y2)))

                    if x2_i <= x1_i or y2_i <= y1_i:
                        self.get_logger().warn(
                            "Invalid bbox size after clamping; skipping depth for this object."
                        )
                    else:
                        # Extract ROI of depth; use median of valid (non-zero, non-nan) values
                        depth_roi = self.latest_depth[y1_i : y2_i + 1, x1_i : x2_i + 1]
                        if depth_roi.size == 0:
                            pass
                        else:
                            valid_mask = np.isfinite(depth_roi) & (
                                depth_roi > 0.001
                            )  # ignore zeros/near-zero
                            if np.any(valid_mask):
                                z = float(np.median(depth_roi[valid_mask]))
                                # Use bbox center for x,y projection
                                u = int((x1 + x2) / 2.0)
                                v = int((y1 + y2) / 2.0)
                                # clamp center to image bounds
                                u = max(0, min(w - 1, u))
                                v = max(0, min(h - 1, v))

                                # If the particular center pixel is invalid, you could fallback to roi median - we already have z
                                # Convert pixel + z -> camera coords
                                x = (u - cx) * z / fx
                                y = (v - cy) * z / fy
                                obj.position = [float(x), float(y), float(z)]
                                position_3d = (x, y, z)
                                # Small covariance estimate based on bbox size and depth (naive)
                                px_width = max(1, x2_i - x1_i)
                                px_height = max(1, y2_i - y1_i)
                                # crude variance estimates:
                                var_xy = (
                                    ((px_width + px_height) / 2.0) * (z / fx) * 0.005
                                )
                                var_xy = max(var_xy, 1e-6)
                                obj.position_covariance = [
                                    var_xy,
                                    0.0,
                                    0.0,
                                    0.0,
                                    var_xy,
                                    0.0,
                                ]
                                obj.tracking_available = True
                                self.get_logger().debug(
                                    f"3D position for {class_label}: x={x:.2f}, y={y:.2f}, z={z:.2f}"
                                )
                            else:
                                # no valid depth in roi
                                obj.tracking_available = False
                else:
                    # no depth or no intrinsics
                    obj.tracking_available = False

                # Fill minimal 3D bbox / skeleton defaults (so message fields exist)
                try:
                    # Many of these sub-messages exist; set basic values
                    obj.dimensions_3d = [0.0, 0.0, 0.0]
                    obj.skeleton_available = False
                    obj.body_format = 0
                    obj.head_position = [0.0, 0.0, 0.0]
                except Exception:
                    pass

                # Append to list
                cone_objects.append(obj)

                # Store info for visualization
                detections_info.append(
                    {
                        "bbox": (int(x1), int(y1), int(x2), int(y2)),
                        "label": class_label,
                        "confidence": confidence,
                        "position_3d": position_3d,
                    }
                )

        return cone_objects, detections_info

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
