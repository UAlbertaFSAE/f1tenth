"""Object Detection node (just testing .pt models for now), will likely have to
integrate custom object detector into ros2 wrapper, or role my own shit idk

TODO:
- build mock image generator node
- write annotated result images to a file to check results
- test cutting image size and stuff to see if you can get inference speed down
"""

import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO


class ObjectDetectorNode(Node):
    """object detection node, reads data from ZED ROS2 wrapper and runs inference
    on the images"""

    def __init__(self):
        super().__init__("object_detector_node")
        self.declare_parameter("cam_info_topic", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")

        self.cam_info_topic = self.get_parameter("cam_info_topic").value
        self.image_topic = self.get_parameter("image_topic").value

        print(self.cam_info_topic)
        print(self.image_topic)

        # load in model configuration information
        with open(
            "/f1tenth/dev_ws/src/perception/test_object_detection/params/model_config.yaml"
        ) as f:
            model_config = yaml.safe_load(f)
            self.model = YOLO(model_config["pt_models"]["yolov8s_best"])
            self.conf_thresh = model_config["inference_params"]["conf"]
            self.iou_thresh = model_config["inference_params"]["iou"]

        self.video_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            self.video_qos,
        )

    def image_callback(self, img_msg):
        """TODO: add appropriate type hints"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            print(cv_image.shape)
            results = self.model(cv_image)
            print(results)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    """Run the object detector node"""
    rclpy.init(args=args)
    object_detector_node = ObjectDetectorNode()
    rclpy.spin(object_detector_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
