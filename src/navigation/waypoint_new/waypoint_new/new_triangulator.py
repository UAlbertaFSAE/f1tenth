#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseStamped
from rc_interfaces.msg import Cone, Cones

import math


class ConeWithDistance:
    """Helper class to store cone with its distance and angle from vehicle"""

    def __init__(self, cone, distance, angle):
        self.cone = cone
        self.distance = distance
        self.angle = angle


class Triangulator(Node):
    def __init__(self):
        super().__init__("triangulator")

        # Parameters / topics
        self.declare_parameter("cones_topic", "/detection_generator/cone_data")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("waypoint_topic", "/waypoints")
        self.declare_parameter("path_topic", "/planned_path")

        self.declare_parameter("lookahead_distance", 10.0)
        self.declare_parameter("num_waypoints", 5)
        self.declare_parameter("min_track_width", 2.0)
        self.declare_parameter("max_track_width", 6.0)

        # New parameters for start gate
        self.declare_parameter("use_start_gate", True)
        self.declare_parameter("start_colors", ["orange", "large_orange"])
        self.declare_parameter(
            "start_gate_max_width", 8.0
        )  # allow a bit wider for start gate

        cones_topic = self.get_parameter("cones_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        waypoint_topic = self.get_parameter("waypoint_topic").value
        path_topic = self.get_parameter("path_topic").value

        self.min_track_width = float(self.get_parameter("min_track_width").value)
        self.max_track_width = float(self.get_parameter("max_track_width").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.num_waypoints = int(self.get_parameter("num_waypoints").value)

        self.use_start_gate = bool(self.get_parameter("use_start_gate").value)
        self.start_colors = list(self.get_parameter("start_colors").value)
        self.start_gate_max_width = float(
            self.get_parameter("start_gate_max_width").value
        )

        # Subscribers
        self.cones_sub = self.create_subscription(
            Cones, cones_topic, self.cones_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        # Publishers
        self.waypoint_pub = self.create_publisher(Point, waypoint_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        self.latest_odom = None

        self.get_logger().info("Triangulator node (Transformer-integrated)")
        self.get_logger().info(f"Cones topic: {cones_topic}")
        self.get_logger().info(f"Odom topic: {odom_topic}")
        self.get_logger().info(
            f"Lookahead: {self.lookahead_distance}m, Waypoints: {self.num_waypoints}"
        )
        self.get_logger().info(
            f"Start gate enabled: {self.use_start_gate}, start colors: {self.start_colors}"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def cones_callback(self, msg: Cones):
        self.get_logger().info(f"Received cone data with {len(msg.cones)} cones")

        if self.latest_odom is None:
            self.get_logger().warn("Odometry not received yet. Skipping processing.")
            return

        vehicle_pos = self.latest_odom.pose.pose.position
        vehicle_heading = self.calculate_heading_from_quaternion(
            self.latest_odom.pose.pose.orientation
        )

        # Try START gate first (orange / large_orange) ---
        start_midpoint = None
        if self.use_start_gate:
            start_midpoint = self.try_start_gate(msg, vehicle_pos, vehicle_heading)

        if start_midpoint is not None:
            # Publish start waypoint immediately
            self.waypoint_pub.publish(start_midpoint)
            self.get_logger().info(
                f"Published START waypoint (orange gate): ({start_midpoint.x:.2f}, {start_midpoint.y:.2f})"
            )

            # Also publish a Path (start-only path)
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "odom"  # transformer outputs in odom

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = start_midpoint
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info("Published start path with 1 waypoint")
            return

        # --- 2) Normal triangulation with BLUE/YELLOW boundaries ---
        left_cones = self.extract_cones_by_color(msg, "blue")
        right_cones = self.extract_cones_by_color(msg, "yellow")

        # fallback: closest blue/yellow if pairing fails
        left_cone = self.find_closest_cone(msg, "blue", vehicle_pos)
        right_cone = self.find_closest_cone(msg, "yellow", vehicle_pos)

        if not left_cones or not right_cones:
            if left_cone and right_cone:
                self.get_logger().info(
                    "Using fallback: single closest blue/yellow cone pair"
                )
                waypoint = self.calculate_midpoint(left_cone, right_cone)
                self.waypoint_pub.publish(waypoint)
                self.get_logger().info(
                    f"Published fallback waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f})"
                )
            else:
                self.get_logger().warn("No cones available for fallback waypoint")
            return

        sorted_left = self.filter_and_sort_cones(
            left_cones, vehicle_pos, vehicle_heading
        )
        sorted_right = self.filter_and_sort_cones(
            right_cones, vehicle_pos, vehicle_heading
        )

        cone_pairs = self.match_cone_pairs(sorted_left, sorted_right)

        if not cone_pairs:
            self.get_logger().warn("Could not match any valid cone pairs")
            return

        self.get_logger().info(f"Matched {len(cone_pairs)} cone pairs (gates)")

        waypoints = []
        for left, right in cone_pairs:
            waypoints.append(self.calculate_midpoint(left, right))

        if not waypoints:
            self.get_logger().warn("No waypoints generated")
            return

        # Publish the first waypoint
        self.waypoint_pub.publish(waypoints[0])
        self.get_logger().info(
            f"Published waypoint: ({waypoints[0].x:.2f}, {waypoints[0].y:.2f}, {waypoints[0].z:.2f})"
        )

        # Publish full path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"  # transformer outputs in odom

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = waypoint
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(waypoints)} waypoints")

    # START gate logic (orange)

    def try_start_gate(self, msg: Cones, vehicle_pos, vehicle_heading):

        # Find a pair of start cones (orange/large_orange) that form a plausible gate
        # and return their midpoint. Returns None if no suitable gate found.

        start_cones = []
        for cone in msg.cones:
            if cone.color in self.start_colors:
                start_cones.append(cone)

        if len(start_cones) < 2:
            return None

        # Filter to cones in front and within lookahead distance
        filtered = []
        for cone in start_cones:
            distance = math.hypot(cone.x - vehicle_pos.x, cone.y - vehicle_pos.y)
            dx = cone.x - vehicle_pos.x
            dy = cone.y - vehicle_pos.y
            cone_angle = math.atan2(dy, dx)
            angle = self.normalize_angle(cone_angle - vehicle_heading)

            if distance <= self.lookahead_distance and abs(angle) < math.pi / 2.0:
                filtered.append(cone)

        if len(filtered) < 2:
            return None

        # Choose the "best" pair:

        best_pair = None
        best_score = float("inf")

        for i in range(len(filtered)):
            for j in range(i + 1, len(filtered)):
                c1 = filtered[i]
                c2 = filtered[j]
                width = math.hypot(c1.x - c2.x, c1.y - c2.y)

                if width > self.start_gate_max_width:
                    continue

                d1 = math.hypot(c1.x - vehicle_pos.x, c1.y - vehicle_pos.y)
                d2 = math.hypot(c2.x - vehicle_pos.x, c2.y - vehicle_pos.y)
                score = (d1 + d2) / 2.0

                if score < best_score:
                    best_score = score
                    best_pair = (c1, c2)

        if best_pair is None:
            return None

        return self.calculate_midpoint(best_pair[0], best_pair[1])

    # Cone utilities (Cones msg)

    def extract_cones_by_color(self, msg: Cones, color: str):
        cones = []
        for cone in msg.cones:
            if cone.color == color:
                cones.append(cone)
        return cones

    def find_closest_cone(self, msg: Cones, color: str, vehicle_pos):
        min_distance = float("inf")
        result = None
        for cone in msg.cones:
            if cone.color != color:
                continue
            dist = math.sqrt(
                ((cone.x - vehicle_pos.x) ** 2) + ((cone.y - vehicle_pos.y) ** 2)
            )
            if dist < min_distance:
                min_distance = dist
                result = cone
        return result

    def filter_and_sort_cones(self, cones, position, heading):
        cones_with_dist = []
        for cone in cones:
            distance = math.hypot(cone.x - position.x, cone.y - position.y)

            dx = cone.x - position.x
            dy = cone.y - position.y
            cone_angle = math.atan2(dy, dx)
            angle = self.normalize_angle(cone_angle - heading)

            if distance <= self.lookahead_distance and abs(angle) < math.pi / 2.0:
                cones_with_dist.append(ConeWithDistance(cone, distance, angle))

        cones_with_dist.sort(key=lambda c: c.distance)
        return cones_with_dist

    def match_cone_pairs(self, left_cones, right_cones):
        pairs = []
        left_idx = 0
        right_idx = 0

        while (
            left_idx < len(left_cones)
            and right_idx < len(right_cones)
            and len(pairs) < self.num_waypoints
        ):
            left = left_cones[left_idx]
            right = right_cones[right_idx]

            if self.is_valid_gate(left.cone, right.cone):
                pairs.append((left.cone, right.cone))
                left_idx += 1
                right_idx += 1
            elif left.distance < right.distance:
                left_idx += 1
            else:
                right_idx += 1

        return pairs

    def is_valid_gate(self, left_cone: Cone, right_cone: Cone):
        distance_between = math.sqrt(
            ((left_cone.x - right_cone.x) ** 2) + ((left_cone.y - right_cone.y) ** 2)
        )
        return self.min_track_width <= distance_between <= self.max_track_width

    def calculate_midpoint(self, cone1: Cone, cone2: Cone):
        midpoint = Point()
        midpoint.x = (cone1.x + cone2.x) / 2.0
        midpoint.y = (cone1.y + cone2.y) / 2.0
        midpoint.z = (getattr(cone1, "z", 0.0) + getattr(cone2, "z", 0.0)) / 2.0
        return midpoint

    def calculate_heading_from_quaternion(self, quat):
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = Triangulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
