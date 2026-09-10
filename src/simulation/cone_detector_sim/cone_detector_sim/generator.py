# MIT License

# Copyright (c) 2026 Krupal Shah

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from rc_interfaces.msg import Cone, Cones
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from visualization_msgs.msg import Marker, MarkerArray

COLOR_RGB = {
    "blue": (0.0, 0.0, 1.0),
    "yellow": (1.0, 1.0, 0.0),
}


@dataclass
class Station:
    """One cone pair (a left/blue and right/yellow cone) along the track."""

    station_id: int
    blue: tuple
    yellow: tuple
    center: tuple


class ConeDetectorSim(Node):
    """Publish the cones currently visible ahead of the car, driven by odometry.

    Each odometry update recomputes the visible cone set fresh from the car's
    real position and heading (nearest station lookup + forward distance/FOV
    window) - there is no pre-baked per-frame timeline, so publishing tracks
    however fast or slow the car is actually moving. This simulates what a
    real cone detector (camera/LiDAR clustering) would output; the ground
    truth for the whole track lives separately in map_generator's /track_map.
    """

    CSV_FIELD_COUNT = 4

    def __init__(self) -> None:
        """Initialize parameters, publishers, odometry subscription, and station list."""
        super().__init__("cone_detector_sim_node")

        self.declare_parameter("csv_path", "")
        self.declare_parameter("cone_topic", "/cone_positions")
        self.declare_parameter("cone_marker_topic", "/cone_view_markers")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("loop_track", True)
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("min_visible_dist", 0.3)
        self.declare_parameter("max_visible_dist", 10.0)
        self.declare_parameter("fov_half_deg", 55.0)
        self.declare_parameter("arc_window_behind_m", 1.0)
        self.declare_parameter("arc_window_ahead_m", 12.0)
        self.declare_parameter("min_publish_period_s", 0.1)
        self.declare_parameter("view_persist", False)

        configured_csv_path = (
            self.get_parameter("csv_path").get_parameter_value().string_value
        )
        cone_topic = self.get_parameter("cone_topic").get_parameter_value().string_value
        cone_marker_topic = (
            self.get_parameter("cone_marker_topic").get_parameter_value().string_value
        )
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.track_closed = (
            self.get_parameter("loop_track").get_parameter_value().bool_value
        )
        qos_depth = self.get_parameter("qos_depth").get_parameter_value().integer_value
        self.min_visible_dist = (
            self.get_parameter("min_visible_dist").get_parameter_value().double_value
        )
        self.max_visible_dist = (
            self.get_parameter("max_visible_dist").get_parameter_value().double_value
        )
        self.fov_half_rad = math.radians(
            self.get_parameter("fov_half_deg").get_parameter_value().double_value
        )
        self.arc_window_behind_m = (
            self.get_parameter("arc_window_behind_m").get_parameter_value().double_value
        )
        self.arc_window_ahead_m = (
            self.get_parameter("arc_window_ahead_m").get_parameter_value().double_value
        )
        self.min_publish_period_s = (
            self.get_parameter("min_publish_period_s")
            .get_parameter_value()
            .double_value
        )
        self.view_persist = (
            self.get_parameter("view_persist").get_parameter_value().bool_value
        )

        if qos_depth < 1:
            self.get_logger().warn("qos_depth must be >= 1. Falling back to 10")
            qos_depth = 10

        csv_path = self.resolve_csv_path(configured_csv_path)
        self.stations = self.read_csv(csv_path)

        self.cone_publisher = self.create_publisher(Cones, cone_topic, int(qos_depth))
        self.marker_publisher = self.create_publisher(
            MarkerArray, cone_marker_topic, int(qos_depth)
        )
        self.last_publish_time: RclpyTime | None = None
        self.persist_frame_counter = 0
        self.odom_subscription = self.create_subscription(
            Odometry, odom_topic, self.on_odometry, int(qos_depth)
        )

        self.get_logger().info(
            f"Loaded {len(self.stations)} cone stations from {csv_path}"
        )
        self.get_logger().info(
            f"Publishing visible cones on {cone_topic} (markers on {cone_marker_topic}), "
            f"driven by odometry on {odom_topic}, view_persist={self.view_persist}"
        )

    def on_odometry(self, msg: Odometry) -> None:
        """Recompute and publish the cones currently visible from the car's pose."""
        if not self.stations:
            return

        now = self.get_clock().now()
        if self.last_publish_time is not None:
            elapsed = (now - self.last_publish_time).nanoseconds / 1e9
            if elapsed < self.min_publish_period_s:
                return
        self.last_publish_time = now

        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        car_heading = self.yaw_from_quaternion(msg.pose.pose.orientation)

        visible = self.visible_stations((car_x, car_y), car_heading)

        cones = Cones()
        for station in visible:
            for point, color in ((station.blue, "blue"), (station.yellow, "yellow")):
                cone = Cone()
                cone.x = point[0]
                cone.y = point[1]
                cone.color = color
                cones.cones.append(cone)

        self.cone_publisher.publish(cones)
        self.marker_publisher.publish(self.build_cone_markers(visible, now))
        self.get_logger().debug(
            f"Published {len(cones.cones)} cones from pose ({car_x:.2f}, {car_y:.2f})"
        )

    def build_cone_markers(self, visible: list, stamp: Time) -> MarkerArray:
        """Sphere markers for the currently visible blue/yellow cones.

        view_persist=false: DELETEALL first, then fixed ids (0=blue, 1=yellow)
        so each frame replaces the last. view_persist=true: skip the delete
        and use a per-frame namespace so old frames' cones stay on screen.
        """
        marker_array = MarkerArray()

        if not self.view_persist:
            clear = Marker()
            clear.header.frame_id = self.frame_id
            clear.header.stamp = stamp.to_msg()
            clear.action = Marker.DELETEALL
            marker_array.markers.append(clear)
            ns_suffix = ""
        else:
            ns_suffix = f"_{self.persist_frame_counter}"
            self.persist_frame_counter += 1

        for color, (r, g, b) in COLOR_RGB.items():
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp.to_msg()
            marker.ns = f"cone_view_{color}{ns_suffix}"
            marker.id = 0
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.25
            marker.color.r, marker.color.g, marker.color.b = r, g, b
            marker.color.a = 1.0
            for station in visible:
                point = station.blue if color == "blue" else station.yellow
                p = Point()
                p.x, p.y = point[0], point[1]
                marker.points.append(p)
            marker_array.markers.append(marker)

        return marker_array

    def visible_stations(self, car_pos: tuple, car_heading: float) -> list:
        """Nearest-station lookup + arc-length window + forward/FOV filter."""
        nearest_index = min(
            range(len(self.stations)),
            key=lambda i: self.distance(self.stations[i].center, car_pos),
        )

        window_indices = self.arc_length_window(nearest_index)

        forward = (math.cos(car_heading), math.sin(car_heading))
        left = (-math.sin(car_heading), math.cos(car_heading))

        visible = []
        for index in window_indices:
            station = self.stations[index]
            rel = (station.center[0] - car_pos[0], station.center[1] - car_pos[1])
            fwd = rel[0] * forward[0] + rel[1] * forward[1]
            lat = rel[0] * left[0] + rel[1] * left[1]
            if fwd < self.min_visible_dist or fwd > self.max_visible_dist:
                continue
            if abs(math.atan2(lat, fwd)) > self.fov_half_rad:
                continue
            visible.append((fwd, station))

        visible.sort(key=lambda item: item[0])
        return [station for _, station in visible]

    def arc_length_window(self, nearest_index: int) -> list:
        """Station indices within arc_window_behind_m/arc_window_ahead_m of nearest_index."""
        count = len(self.stations)
        indices = [nearest_index]

        accumulated = 0.0
        prev_index = nearest_index
        index = nearest_index
        while accumulated < self.arc_window_ahead_m:
            next_index = index + 1
            if next_index >= count:
                if not self.track_closed:
                    break
                next_index = 0
            if next_index == nearest_index:
                break
            accumulated += self.distance(
                self.stations[index].center, self.stations[next_index].center
            )
            indices.append(next_index)
            index = next_index

        accumulated = 0.0
        index = nearest_index
        while accumulated < self.arc_window_behind_m:
            prev_index = index - 1
            if prev_index < 0:
                if not self.track_closed:
                    break
                prev_index = count - 1
            if prev_index == nearest_index:
                break
            accumulated += self.distance(
                self.stations[index].center, self.stations[prev_index].center
            )
            indices.append(prev_index)
            index = prev_index

        return indices

    @staticmethod
    def distance(a: tuple, b: tuple) -> float:
        """Euclidean distance between two (x, y) points."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def yaw_from_quaternion(orientation: Quaternion) -> float:
        """Yaw angle in radians from a quaternion orientation."""
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def resolve_csv_path(self, configured_path: str) -> str:
        """Resolve configured cone CSV path, falling back to a bundled sample."""
        share_dir = Path(get_package_share_directory("cone_detector_sim"))

        if configured_path:
            configured = Path(configured_path)
            if configured.is_absolute():
                return str(configured)
            return str(share_dir / configured)

        return str(share_dir / "data" / "straight.csv")

    def read_csv(self, path: str) -> list:
        """Read track CSV rows and group cones into ordered stations by id."""
        csv_path = Path(path)
        if not csv_path.exists() or not csv_path.is_file():
            self.get_logger().error(f"Failed to open track csv: {path}")
            return []

        self.get_logger().info(f"Opened track csv: {path}")

        cones_by_id: dict[int, dict[str, tuple]] = {}
        order: list[int] = []

        with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                if not row:
                    continue

                if len(row) != self.CSV_FIELD_COUNT:
                    self.get_logger().warn(
                        "Skipping malformed row in %s. Expected %d fields, got %d: %s",
                        path,
                        self.CSV_FIELD_COUNT,
                        len(row),
                        ",".join(row),
                    )
                    continue

                station_text = row[0].strip()
                if not station_text:
                    continue

                if not (station_text[0].isdigit() or station_text[0] == "-"):
                    continue

                try:
                    station_id = int(station_text)
                    x = float(row[1].strip())
                    y = float(row[2].strip())
                    color = row[3].strip()
                except ValueError:
                    self.get_logger().warn(
                        "Skipping non-numeric row in %s: %s", path, ",".join(row)
                    )
                    continue

                if not color:
                    self.get_logger().warn(
                        "Skipping row with empty cone color in %s: %s",
                        path,
                        ",".join(row),
                    )
                    continue

                if station_id not in cones_by_id:
                    cones_by_id[station_id] = {}
                    order.append(station_id)
                cones_by_id[station_id][color] = (x, y)

        stations = []
        for station_id in order:
            colors = cones_by_id[station_id]
            blue = colors.get("blue")
            yellow = colors.get("yellow")
            if blue is None or yellow is None:
                self.get_logger().warn(
                    "Skipping incomplete station id=%d in %s (missing blue or yellow cone)",
                    station_id,
                    path,
                )
                continue
            center = ((blue[0] + yellow[0]) / 2.0, (blue[1] + yellow[1]) / 2.0)
            stations.append(
                Station(station_id=station_id, blue=blue, yellow=yellow, center=center)
            )

        return stations


def main(args: list[str] | None = None) -> None:
    """Run the simulated cone detector node."""
    rclpy.init(args=args)
    node = ConeDetectorSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
