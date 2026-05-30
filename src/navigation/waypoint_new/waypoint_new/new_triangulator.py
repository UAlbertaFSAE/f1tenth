#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

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
        self.declare_parameter("min_track_width", 0.01)
        self.declare_parameter("max_track_width", 6.0)
        self.declare_parameter("use_spline", True)
        self.declare_parameter("spline_samples_per_segment", 6)
        self.declare_parameter("spline_min_points", 4)
        self.declare_parameter("track_point_min_spacing", 0.5)
        self.declare_parameter("track_max_points", 2000)
        self.declare_parameter("track_closure_distance", 1.0)
        self.declare_parameter("track_min_points_for_loop", 30)
        self.declare_parameter("publish_point_count", 20)
        self.declare_parameter("publish_spacing", 0.5)

        # Start gate parameters (string colors, since Cone.msg says `string color`)
        self.declare_parameter("use_start_gate", False)
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
        self.use_spline = bool(self.get_parameter("use_spline").value)
        self.spline_samples_per_segment = int(
            self.get_parameter("spline_samples_per_segment").value)
        self.spline_min_points = int(
            self.get_parameter("spline_min_points").value)
        self.track_point_min_spacing = float(
            self.get_parameter("track_point_min_spacing").value)
        self.track_max_points = int(
            self.get_parameter("track_max_points").value)
        self.track_closure_distance = float(
            self.get_parameter("track_closure_distance").value)
        self.track_min_points_for_loop = int(
            self.get_parameter("track_min_points_for_loop").value)
        self.publish_point_count = int(
            self.get_parameter("publish_point_count").value)
        self.publish_spacing = float(
            self.get_parameter("publish_spacing").value)

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
        self.track_points: List[Point] = []
        self.track_closed = False

        self.get_logger().info("Triangulator node started")
        self.get_logger().info(f"Cones topic: {cones_topic}")
        self.get_logger().info(f"Odom topic: {odom_topic}")
        self.get_logger().info(
            f"Lookahead={self.lookahead_distance}m num_waypoints={self.num_waypoints} "
            f"min_width={self.min_track_width} max_width={self.max_track_width}"
        )
        self.get_logger().info(
            f"Spline enabled={self.use_spline} samples_per_segment={self.spline_samples_per_segment} "
            f"min_points={self.spline_min_points} min_spacing={self.track_point_min_spacing} "
            f"publish_count={self.publish_point_count} publish_spacing={self.publish_spacing} "
            f"loop_close_dist={self.track_closure_distance} loop_min_points={self.track_min_points_for_loop}"
        )
        self.get_logger().info(
            f"Start gate enabled={self.use_start_gate} start_colors={self.start_colors} "
            f"start_gate_max_width={self.start_gate_max_width}"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    # This function is the function that calls all the other functions where it matches the cone pairs first
    # and extracts the position with odom data.
    def cones_callback(self, msg: Cones):
        self.get_logger().info(
            f"Received cone data with {len(msg.cones)} cones")

        if self.latest_odom is None:
            self.get_logger().warn("Odometry not received yet. Skipping processing.")
            return

        vehicle_pos = self.latest_odom.pose.pose.position
        vehicle_heading = self.calculate_heading_from_quaternion(
            self.latest_odom.pose.pose.orientation)

        # Debug statment to make sure the colors are seen
        colors_seen = sorted({str(c.color) for c in msg.cones})
        self.get_logger().info(
            f"vehicle_pos=({vehicle_pos.x:.2f},{vehicle_pos.y:.2f}) "
            f"heading_deg={math.degrees(vehicle_heading):.1f} colors_seen={colors_seen}"
        )
        # show first 10 detected cones
        for i, c in enumerate(msg.cones[:10]):
            self.get_logger().info(
                f"raw[{i}] color='{c.color}' pos=({c.x:.3f},{c.y:.3f})")

        # tries the start gate before looking and matching blue/yellow cone pairs
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

        midpoints: List[Point] = []
        if not left_cones or not right_cones:
            if left_cone and right_cone:
                self.get_logger().warn("No lists for pairing; using fallback closest blue/yellow")
                midpoints = [self.calculate_midpoint(left_cone, right_cone)]
            else:
                self.get_logger().warn("No cones available for fallback waypoint")
        else:
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

            if cone_pairs:
                self.get_logger().info(
                    f"Matched {len(cone_pairs)} cone pairs (gates)")
                midpoints = [self.calculate_midpoint(
                    left, right) for (left, right) in cone_pairs]
            else:
                self.get_logger().warn("Could not match any valid cone pairs")

        if midpoints:
            self.update_track_points(midpoints, vehicle_pos, vehicle_heading)
            self.update_track_closure()

        if self.use_spline:
            spline_points = self.build_spline_points()
            if spline_points:
                publish_points = self.sample_spline_points_ahead(
                    spline_points, vehicle_pos, vehicle_heading)
                if publish_points:
                    self.publish_waypoints(publish_points)
                    self.publish_path(spline_points)
                    return

        if not midpoints:
            self.get_logger().warn("No waypoints generated")
            return

        # NOTE: Previously this published waypoints[0] (closest gate).
        # On curves / end-of-visibility, that tends to straighten the car.
        # Publishing the farthest gate encourages committing to the turn.
        target_wp = midpoints[-1]

        self.waypoint_pub.publish(target_wp)
        self.get_logger().info(
            f"Published waypoint: ({target_wp.x:.2f}, {target_wp.y:.2f}, {target_wp.z:.2f})"
        )

        self.publish_path(midpoints)

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
            dx = cone.x - vehicle_pos.x
            dy = cone.y - vehicle_pos.y
            distance = math.sqrt((dx * dx) + (dy * dy))

            cone_angle = math.atan2(dy, dx)
            angle = self.normalize_angle(cone_angle - vehicle_heading)

            passes = distance <= self.lookahead_distance

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

                wdx = c1.x - c2.x
                wdy = c1.y - c2.y
                width = math.sqrt((wdx * wdx) + (wdy * wdy))

                if width > self.start_gate_max_width:
                    self.get_logger().info(
                        f"[start-pair-skip] width={width:.3f} > max={self.start_gate_max_width:.3f}"
                    )
                    continue

                d1x = c1.x - vehicle_pos.x
                d1y = c1.y - vehicle_pos.y
                d1 = math.sqrt((d1x * d1x) + (d1y * d1y))

                d2x = c2.x - vehicle_pos.x
                d2y = c2.y - vehicle_pos.y
                d2 = math.sqrt((d2x * d2x) + (d2y * d2y))

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

    # Cone utilities
    def extract_cones_by_color(self, msg: Cones, color: str):
        cones = [cone for cone in msg.cones if cone.color == color]
        return cones

    # finds closest left or right cone pair depending on distance
    def find_closest_cone(self, msg: Cones, color: str, vehicle_pos):
        min_distance = float("inf")
        result = None
        for cone in msg.cones:
            if cone.color != color:
                continue

            dx = cone.x - vehicle_pos.x
            dy = cone.y - vehicle_pos.y
            dist = math.sqrt((dx * dx) + (dy * dy))

            if dist < min_distance:
                min_distance = dist
                result = cone
        return result
    # makes sure its less than lookahead distance

    def filter_and_sort_cones(self, cones, position, heading):
        cones_with_dist = []
        for cone in cones:
            dx = cone.x - position.x
            dy = cone.y - position.y
            distance = math.sqrt((dx * dx) + (dy * dy))

            cone_angle = math.atan2(dy, dx)
            angle = self.normalize_angle(cone_angle - heading)

            passes = distance <= self.lookahead_distance

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

        # NEW pairing logic:
        # The old two-pointer approach (closest-left with closest-right) can fail on curves
        # and cause the car to "straighten out" between cones near the end of visibility.
        # This pairs each left cone to the best matching right cone that forms a valid gate.
        used_right = set()

        left_list = [c.cone for c in left_cones]
        right_list = [c.cone for c in right_cones]

        for left in left_list:
            best_right_idx = None
            best_right = None
            best_score = float("inf")

            for idx, right in enumerate(right_list):
                if idx in used_right:
                    continue

                valid = self.is_valid_gate(left, right)

                if not valid:
                    continue

                wdx = left.x - right.x
                wdy = left.y - right.y
                width = math.sqrt((wdx * wdx) + (wdy * wdy))

                # Score based on gate width (stable on curves).
                score = width

                self.get_logger().info(
                    f"[pair-attempt] L=({left.x:.3f},{left.y:.3f}) "
                    f"R=({right.x:.3f},{right.y:.3f}) "
                    f"width={width:.3f} valid={valid} score={score:.3f}"
                )

                if score < best_score:
                    best_score = score
                    best_right_idx = idx
                    best_right = right

            if best_right is not None:
                used_right.add(best_right_idx)
                pairs.append((left, best_right))

            if len(pairs) >= self.num_waypoints:
                break

        return pairs

    def is_valid_gate(self, left_cone: Cone, right_cone: Cone):
        dx = left_cone.x - right_cone.x
        dy = left_cone.y - right_cone.y
        distance_between = math.sqrt((dx * dx) + (dy * dy))
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

    def update_track_points(self, new_points: List[Point], vehicle_pos, vehicle_heading):
        if not new_points:
            return

        heading_vec = (math.cos(vehicle_heading), math.sin(vehicle_heading))
        candidates = [
            point for point in new_points
            if self.is_point_ahead(vehicle_pos, heading_vec, point)
        ]

        if not candidates:
            return

        target = max(candidates, key=lambda p: self.point_distance(p, vehicle_pos))
        if not self.track_points:
            self.track_points.append(target)
        else:
            if self.point_distance(target, self.track_points[-1]) >= self.track_point_min_spacing:
                self.track_points.append(target)
                if len(self.track_points) > self.track_max_points:
                    self.track_points.pop(0)

    def update_track_closure(self):
        if self.track_closed:
            return
        if len(self.track_points) < self.track_min_points_for_loop:
            return
        if self.point_distance(self.track_points[0], self.track_points[-1]) <= self.track_closure_distance:
            self.track_closed = True
            self.get_logger().info("Track closure detected; spline will wrap.")

    def build_spline_points(self) -> List[Point]:
        if len(self.track_points) < self.spline_min_points:
            return []

        samples = max(1, self.spline_samples_per_segment)
        points = self.track_points
        spline_points: List[Point] = []

        if self.track_closed:
            count = len(points)
            for i in range(count):
                p0 = points[(i - 1) % count]
                p1 = points[i % count]
                p2 = points[(i + 1) % count]
                p3 = points[(i + 2) % count]
                for step in range(samples):
                    t = step / float(samples)
                    spline_points.append(self.catmull_rom_point(p0, p1, p2, p3, t))
        else:
            padded = [points[0]] + points + [points[-1]]
            for i in range(1, len(padded) - 2):
                p0, p1, p2, p3 = padded[i - 1], padded[i], padded[i + 1], padded[i + 2]
                for step in range(samples):
                    t = step / float(samples)
                    spline_points.append(self.catmull_rom_point(p0, p1, p2, p3, t))
            spline_points.append(points[-1])

        return spline_points

    def sample_spline_points_ahead(
        self,
        spline_points: List[Point],
        vehicle_pos,
        vehicle_heading,
    ) -> List[Point]:
        if not spline_points:
            return []

        heading_vec = (math.cos(vehicle_heading), math.sin(vehicle_heading))
        start_idx = self.find_nearest_point_index(spline_points, vehicle_pos)
        if start_idx is None:
            return []

        start_idx = self.find_ahead_index(
            spline_points, start_idx, vehicle_pos, heading_vec)
        if start_idx is None:
            return []

        result = [spline_points[start_idx]]
        if self.publish_point_count <= 1:
            return result

        n = len(spline_points)
        current_idx = start_idx
        spacing_target = max(0.0, self.publish_spacing)
        dist_since = 0.0
        steps = 0
        max_steps = n + 1 if self.track_closed else n

        while len(result) < self.publish_point_count and steps < max_steps:
            next_idx = current_idx + 1
            if next_idx >= n:
                if self.track_closed:
                    next_idx = 0
                else:
                    break

            segment = self.point_distance(spline_points[current_idx], spline_points[next_idx])
            dist_since += segment
            current_idx = next_idx
            steps += 1

            if spacing_target == 0.0 or dist_since >= spacing_target:
                result.append(spline_points[current_idx])
                dist_since = 0.0

        return result

    def find_nearest_point_index(self, points: List[Point], vehicle_pos) -> Optional[int]:
        if not points:
            return None

        best_idx = None
        best_dist = float("inf")
        for i, point in enumerate(points):
            dist = self.point_distance(point, vehicle_pos)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        return best_idx

    def find_ahead_index(
        self,
        points: List[Point],
        start_idx: int,
        vehicle_pos,
        heading_vec: Tuple[float, float],
    ) -> Optional[int]:
        n = len(points)
        if n == 0:
            return None

        idx = start_idx
        for _ in range(n):
            if self.is_point_ahead(vehicle_pos, heading_vec, points[idx]):
                return idx
            idx += 1
            if idx >= n:
                if self.track_closed:
                    idx = 0
                else:
                    break

        return None

    def is_point_ahead(self, vehicle_pos, heading_vec, point: Point) -> bool:
        dx = point.x - vehicle_pos.x
        dy = point.y - vehicle_pos.y
        return (dx * heading_vec[0] + dy * heading_vec[1]) > 0.0

    def point_distance(self, a: Point, b) -> float:
        dx = a.x - b.x
        dy = a.y - b.y
        return math.sqrt((dx * dx) + (dy * dy))

    def catmull_rom_point(self, p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
        t2 = t * t
        t3 = t2 * t

        x = 0.5 * (
            (2.0 * p1.x)
            + (-p0.x + p2.x) * t
            + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2
            + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3
        )
        y = 0.5 * (
            (2.0 * p1.y)
            + (-p0.y + p2.y) * t
            + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2
            + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3
        )

        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        return point

    def publish_waypoints(self, points: List[Point]):
        for point in points:
            self.waypoint_pub.publish(point)

        if points:
            last = points[-1]
            self.get_logger().info(
                f"Published {len(points)} spline waypoints, last=({last.x:.2f}, {last.y:.2f})"
            )

    def publish_path(self, points: List[Point]):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        for point in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = point
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"Published path with {len(points)} waypoints")


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