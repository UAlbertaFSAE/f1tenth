from typing import Any

import rclpy
from geometry_msgs.msg import Point
from rc_interfaces.msg import Cones
from rclpy.node import Node
from visualization_msgs.msg import (
    Marker,
    MarkerArray,
)


class ConesPublisher(Node):
    """This class publishes visialzation topics for the simulator."""

    def __init__(self) -> None:
        """Connect publishers and subscribers with their callbacks."""
        super().__init__("cones_publisher")

        self.subscription = self.create_subscription(
            Cones,
            "/detection_generator/cone_data",
            self.cones_callback,
            10,
        )
        self.publisher_ = self.create_publisher(MarkerArray, "/visible_cones_viz", 10)

        self.allWaypoints: list[Any] = []
        self.subscription = self.create_subscription(
            Point,
            "/waypoint_triangulation/waypoints",
            self.waypoints_callback,
            10,
        )
        self.waypointPub = self.create_publisher(Marker, "/published_waypoint", 10)

        self.get_logger().info("Starting")
        self.subscription  # noqa

    def waypoints_callback(self: Any, msg: Any) -> None:
        """Publishes current waypoint to /published_waypoint visualization topic."""
        self.get_logger().info('I heard: "%s"' % self.allWaypoints)  # noqa
        if [msg.x, msg.y] not in self.allWaypoints:
            self.allWaypoints.append([msg.x, msg.y])
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.waypointPub.publish(marker)

    def cones_callback(self: Any, msg: Any) -> None:
        """Publishes visible cones to /visible_cones_viz visualization topic."""
        self.get_logger().info('I heard: "%s"' % msg.cones)  # noqa
        if len(msg.cones) == 0:
            self.publisher_.publish(MarkerArray())
            return

        allcones = MarkerArray()
        for i, cone in enumerate(msg.cones):
            marker = Marker()
            marker.header.frame_id = (
                "map"  # MUST BE MAP FOR CORRECT COORDINATE REFERENCE
            )
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "my_namespace"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cone.x
            marker.pose.position.y = cone.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0  # Don't forget to set the alpha
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            if cone.color == "blue":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            allcones.markers.append(marker)
        self.publisher_.publish(allcones)


def main(args: Any = None) -> None:
    """Initliaze class and spin up nodes."""
    rclpy.init(args=args)

    viz_subscriber = ConesPublisher()

    rclpy.spin(viz_subscriber)

    viz_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
