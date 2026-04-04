#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseStamped
from rc_interfaces.msg import Cone, Cones


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
        self.declare_parameter("cones_topic", "/cone_transformed")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("waypoint_topic", "/waypoints")
        self.declare_parameter("path_topic", "/planned_path")

        self.declare_parameter("lookahead_distance", 10.0)
        self.declare_parameter("num_waypoints", 5)
        self.declare_parameter("min_track_width", 0.05)
        self.declare_parameter("max_track_width", 6.0)

        # Start gate parameters (string colors, since Cone.msg says `string color`)
        self.declare_parameter("use_start_gate", True)
        self.declare_parameter("start_colors", ["orange", "large_orange"])
        self.declare_parameter("start_gate_max_width", 8.0)

        cones_topic = self.get_parameter("cones_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        waypoint_topic = self.get_parameter("waypoint_topic").value
        path_topic = self.get_parameter("path_topic").value

        self.min_track_width = float(
            self.get_parameter("min_track_width").value)
        self.max_track_width = float(
            self.get_parameter("max_track_width").value)
        self.lookahead_distance = float(
            self.get_parameter("lookahead_distance").value)
        self.num_waypoints = int(self.get_parameter("num_waypoints").value)

        self.use_start_gate = bool(self.get_parameter("use_start_gate").value)
        self.start_colors = list(self.get_parameter("start_colors").value)
        self.start_gate_max_width = float(
            self.get_parameter("start_gate_max_width").value)

        # Subscribers
        self.cones_sub = self.create_subscription(
            Cones, cones_topic, self.cones_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        # Publishers
        self.waypoint_pub = self.create_publisher(Point, waypoint_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        self.latest_odom = None

        self.get_logger().info("Triangulator node started")
        self.get_logger().info(f"Cones topic: {cones_topic}")
        self.get_logger().info(f"Odom topic: {odom_topic}")
        self.get_logger().info(
            f"Lookahead={self.lookahead_distance}m num_waypoints={self.num_waypoints} "
            f"min_width={self.min_track_width} max_width={self.max_track_width}"
        )
        self.get_logger().info(
            f"Start gate enabled={self.use_start_gate} start_colors={self.start_colors} "
            f"start_gate_max_width={self.start_gate_max_width}"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def cones_callback(self, msg: Cones):
        self.get_logger().info(
            f"Received cone data with {len(msg.cones)} cones")

        if self.latest_odom is None:
            self.get_logger().warn("Odometry not received yet. Skipping processing.")
            return

        vehicle_pos = self.latest_odom.pose.pose.position
        vehicle_heading = self.calculate_heading_from_quaternion(
            self.latest_odom.pose.pose.orientation)

        # --- DEBUG: vehicle pose/heading + colors seen
        colors_seen = sorted({str(c.color) for c in msg.cones})
        self.get_logger().info(
            f"vehicle_pos=({vehicle_pos.x:.2f},{vehicle_pos.y:.2f}) "
            f"heading_deg={math.degrees(vehicle_heading):.1f} colors_seen={colors_seen}"
        )
        # show raw cones briefly (first up to 10)
        for i, c in enumerate(msg.cones[:10]):
            self.get_logger().info(
                f"raw[{i}] color='{c.color}' pos=({c.x:.3f},{c.y:.3f})"
            )

        # --- 1) Try START gate first (orange / large_orange)
        start_midpoint = None
        if self.use_start_gate:
            start_midpoint = self.try_start_gate(
                msg, vehicle_pos, vehicle_heading)

        if start_midpoint is not None:
            self.waypoint_pub.publish(start_midpoint)
            self.get_logger().info(
                f"Published START waypoint: ({start_midpoint.x:.2f}, {start_midpoint.y:.2f})"
            )

            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "odom"

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = start_midpoint
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info("Published start path with 1 waypoint")
            return

        # --- 2) Normal triangulation with BLUE/YELLOW boundaries
        left_cones = self.extract_cones_by_color(msg, "blue")
        right_cones = self.extract_cones_by_color(msg, "yellow")

        self.get_logger().info(
            f"Raw extracted: blue={len(left_cones)} yellow={len(right_cones)}")

        # fallback: closest blue/yellow if pairing fails
        left_cone = self.find_closest_cone(msg, "blue", vehicle_pos)
        right_cone = self.find_closest_cone(msg, "yellow", vehicle_pos)

        self.get_logger().info(
            f"Closest cones: blue={left_cone is not None} yellow={right_cone is not None} "
            f"blue_obj={left_cone} yellow_obj={right_cone}"
        )

        if not left_cones or not right_cones:
            if left_cone and right_cone:
                self.get_logger().warn("No lists for pairing; using fallback closest blue/yellow")
                waypoint = self.calculate_midpoint(left_cone, right_cone)
                self.waypoint_pub.publish(waypoint)
                self.get_logger().info(
                    f"Published fallback waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f})"
                )
            else:
                self.get_logger().warn("No cones available for fallback waypoint")
            return

        sorted_left = self.filter_and_sort_cones(
            left_cones, vehicle_pos, vehicle_heading)
        sorted_right = self.filter_and_sort_cones(
            right_cones, vehicle_pos, vehicle_heading)

        self.get_logger().info(
            f"Filtered: blue={len(sorted_left)} yellow={len(sorted_right)}")
        if sorted_left:
            c = sorted_left[0]
            self.get_logger().info(
                f"Closest FILTERED blue: ({c.cone.x:.3f},{c.cone.y:.3f}) dist={c.distance:.3f} angle_deg={math.degrees(c.angle):.1f}"
            )
        if sorted_right:
            c = sorted_right[0]
            self.get_logger().info(
                f"Closest FILTERED yellow: ({c.cone.x:.3f},{c.cone.y:.3f}) dist={c.distance:.3f} angle_deg={math.degrees(c.angle):.1f}"
            )

        cone_pairs = self.match_cone_pairs(sorted_left, sorted_right)
        self.get_logger().info(f"Matched cone pairs: {cone_pairs}")

        if not cone_pairs:
            self.get_logger().warn("Could not match any valid cone pairs")
            return

        self.get_logger().info(f"Matched {len(cone_pairs)} cone pairs (gates)")

        waypoints = [self.calculate_midpoint(
            left, right) for (left, right) in cone_pairs]
        if not waypoints:
            self.get_logger().warn("No waypoints generated")
            return

        self.waypoint_pub.publish(waypoints[0])
        self.get_logger().info(
            f"Published waypoint: ({waypoints[0].x:.2f}, {waypoints[0].y:.2f}, {waypoints[0].z:.2f})"
        )

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = waypoint
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"Published path with {len(waypoints)} waypoints")

    # START gate logic (orange)
    def try_start_gate(self, msg: Cones, vehicle_pos, vehicle_heading):
        start_cones = [
            cone for cone in msg.cones if cone.color in self.start_colors]

        self.get_logger().info(
            f"Start-gate: found {len(start_cones)} cones with colors={self.start_colors}"
        )

        if len(start_cones) < 2:
            return None

        filtered = []
        for cone in start_cones:
            distance = math.hypot(cone.x - vehicle_pos.x,
                                  cone.y - vehicle_pos.y)
            dx = cone.x - vehicle_pos.x
            dy = cone.y - vehicle_pos.y
            cone_angle = math.atan2(dy, dx)
            angle = self.normalize_angle(cone_angle - vehicle_heading)

            passes = distance <= self.lookahead_distance and abs(
                angle) <= (math.pi / 2.0)

            self.get_logger().info(
                f"[start-filter] color={cone.color} pos=({cone.x:.3f},{cone.y:.3f}) "
                f"dist={distance:.3f} angle_deg={math.degrees(angle):.1f} passes={passes}"
            )

            if passes:
                filtered.append(cone)

        if len(filtered) < 2:
            self.get_logger().info("Start-gate: not enough filtered cones in front")
            return None

        best_pair = None
        best_score = float("inf")

        for i in range(len(filtered)):
            for j in range(i + 1, len(filtered)):
                c1 = filtered[i]
                c2 = filtered[j]
                width = math.hypot(c1.x - c2.x, c1.y - c2.y)

                if width > self.start_gate_max_width:
                    self.get_logger().info(
                        f"[start-pair-skip] width={width:.3f} > max={self.start_gate_max_width:.3f}"
                    )
                    continue

                d1 = math.hypot(c1.x - vehicle_pos.x, c1.y - vehicle_pos.y)
                d2 = math.hypot(c2.x - vehicle_pos.x, c2.y - vehicle_pos.y)
                score = (d1 + d2) / 2.0

                self.get_logger().info(
                    f"[start-pair] c1=({c1.x:.3f},{c1.y:.3f}) c2=({c2.x:.3f},{c2.y:.3f}) "
                    f"width={width:.3f} score={score:.3f}"
                )

                if score < best_score:
                    best_score = score
                    best_pair = (c1, c2)

        if best_pair is None:
            self.get_logger().info("Start-gate: no suitable pair found")
            return None

        midpoint = self.calculate_midpoint(best_pair[0], best_pair[1])
        self.get_logger().info(
            f"Start-gate: selected midpoint=({midpoint.x:.3f},{midpoint.y:.3f}) score={best_score:.3f}"
        )
        return midpoint

    # Cone utilities (Cones msg)
    def extract_cones_by_color(self, msg: Cones, color: str):
        cones = [cone for cone in msg.cones if cone.color == color]
        return cones

    def find_closest_cone(self, msg: Cones, color: str, vehicle_pos):
        min_distance = float("inf")
        result = None
        for cone in msg.cones:
            if cone.color != color:
                continue
            dist = math.hypot(cone.x - vehicle_pos.x, cone.y - vehicle_pos.y)
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

            passes = distance <= self.lookahead_distance and abs(
                angle) <= (math.pi / 2.0)

            self.get_logger().info(
                f"[filter] color={cone.color} pos=({cone.x:.3f},{cone.y:.3f}) "
                f"dist={distance:.3f}/{self.lookahead_distance:.3f} "
                f"angle_deg={math.degrees(angle):.1f} passes={passes}"
            )

            if passes:
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

            width = math.hypot(left.cone.x - right.cone.x,
                               left.cone.y - right.cone.y)
            valid = self.is_valid_gate(left.cone, right.cone)

            self.get_logger().info(
                f"[pair-attempt] L=({left.cone.x:.3f},{left.cone.y:.3f}) dL={left.distance:.3f} "
                f"R=({right.cone.x:.3f},{right.cone.y:.3f}) dR={right.distance:.3f} "
                f"width={width:.3f} valid={valid}"
            )

            if valid:
                pairs.append((left.cone, right.cone))
                left_idx += 1
                right_idx += 1
            elif left.distance < right.distance:
                left_idx += 1
            else:
                right_idx += 1

        return pairs

    def is_valid_gate(self, left_cone: Cone, right_cone: Cone):
        distance_between = math.hypot(
            left_cone.x - right_cone.x, left_cone.y - right_cone.y)
        return self.min_track_width <= distance_between <= self.max_track_width

    def calculate_midpoint(self, cone1: Cone, cone2: Cone):
        midpoint = Point()
        midpoint.x = (cone1.x + cone2.x) / 2.0
        midpoint.y = (cone1.y + cone2.y) / 2.0
        # Cone.msg has only x,y,color; keep z at 0.
        midpoint.z = 0.0
        return midpoint

    def calculate_heading_from_quaternion(self, quat):
        # yaw from quaternion
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
