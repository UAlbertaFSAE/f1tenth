"""Publish the full ground-truth track as a latched MarkerArray.

Reads a map_generator cone CSV and publishes left/right boundary lines plus the
centerline. This is the whole track, known ahead of time; distinct from
cone_detector_sim's per-frame limited-FOV view and from the (always blank) /map
OccupancyGrid.
"""

from collections.abc import Sequence
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from map_generator import csv_io


class TrackMapPublisher(Node):
    """Publish a cone CSV as latched boundary and centerline markers."""

    def __init__(self) -> None:
        """Read the CSV named by the csv_path parameter and publish it once."""
        super().__init__("track_map_publisher_node")

        self.declare_parameter("csv_path", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/track_map")

        csv_path = self.get_parameter("csv_path").get_parameter_value().string_value
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        topic = self.get_parameter("topic").get_parameter_value().string_value

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(MarkerArray, topic, qos)

        csv_path = self.resolve_csv_path(csv_path)

        pairs = csv_io.import_csv(csv_path)
        if not pairs:
            self.get_logger().error(f"No cone pairs loaded from {csv_path}")
            return

        marker_array = self._build_markers(pairs)
        self.publisher.publish(marker_array)
        self.get_logger().info(
            f"Published ground-truth track from {csv_path} on {topic} "
            f"({len(pairs)} stations)"
        )

    def resolve_csv_path(self, configured_path: str) -> str:
        """Resolve the configured cone CSV path, falling back to the bundled sample."""
        share_dir = Path(get_package_share_directory("cone_detector_sim"))

        if configured_path:
            configured = Path(configured_path)
            if configured.is_absolute():
                return str(configured)
            return str(share_dir / configured)

        return str(share_dir / "data" / "straight.csv")

    def _build_markers(self, pairs: list) -> MarkerArray:
        blue = [p["blue"] for p in pairs if p["blue"] is not None]
        yellow = [p["yellow"] for p in pairs if p["yellow"] is not None]
        center = csv_io.reconstruct_centerline(pairs)
        now = self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        marker_array.markers.append(
            self._line_strip("left_boundary", 0, blue, (0.0, 0.0, 1.0), now)
        )
        marker_array.markers.append(
            self._line_strip("right_boundary", 1, yellow, (1.0, 1.0, 0.0), now)
        )
        marker_array.markers.append(
            self._line_strip("centerline", 2, center, (0.6, 0.6, 0.6), now)
        )
        return marker_array

    def _line_strip(
        self,
        ns: str,
        marker_id: int,
        points: Sequence[Sequence[float]] | np.ndarray,
        rgb: tuple[float, float, float],
        stamp: Time,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 1.0
        for x, y in points:
            p = Point()
            p.x, p.y = x, y
            marker.points.append(p)
        return marker


def main(args: list[str] | None = None) -> None:
    """Spin the track map publisher node."""
    rclpy.init(args=args)
    node = TrackMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
