import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rc_interfaces.msg import Cone, Cones
from rclpy.node import Node
from scipy.interpolate import splprep, splev




class Spline(Node):
    def __init__(self):
        super().__init__('spline_node')

        self.declare_parameter("cones_topic", "/cone_transformed")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/planned_path_smooth")
        self.declare_parameter("raw_path_topic", "/planned_path")

        self.declare_parameter("lookahead_distance", 10.0)
        self.declare_parameter("max_hop", 5.0)
        self.declare_parameter("spline_smoothing", 0.5)
        self.declare_parameter("num_samples", 20)

        cones_topic = self.get_parameter("cones_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        path_topic = self.get_parameter("path_topic").value
        raw_path_topic = self.get_parameter("raw_path_topic").value

        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.max_hop = float(self.get_parameter("max_hop").value)
        self.spline_smoothing = float(self.get_parameter("spline_smoothing").value)
        self.num_samples = int(self.get_parameter("num_samples").value)

        self.latest_odom: Odometry | None = None
        self.latest_raw_path: Path | None = None

        self.cones_sub = self.create_subscription(
            Cones, cones_topic, self.cones_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.raw_path_sub = self.create_subscription(
            Path, raw_path_topic, self.raw_path_callback, 10
        )

        self.path_pub = self.create_publisher(Path, path_topic, 10)

        self.get_logger().info("Spline node started")

    def odom_callback(self, msg: Odometry):
        """Store the latest odometry message."""
        self.latest_odom = msg

    def raw_path_callback(self, msg: Path):
        """Store the latest raw path from the triangulator."""
        self.latest_raw_path = msg

    def extract_cones_by_color(self, msg: Cones, color: str) -> list[Cone]:
        """Return all cones from a message that match a color string."""
        cones = [cone for cone in msg.cones if cone.color == color]
        return cones

    def find_seed_cone(self, cones: list[Cone], position, heading: float) -> Cone | None:
        """Return the closest cone that is in front of the vehicle."""
        min_distance = float("inf")
        result = None

        for cone in cones:
            dx = cone.x - position.x
            dy = cone.y - position.y
            distance = math.sqrt((dx * dx) + (dy * dy))

            cone_angle = math.atan2(dy, dx)
            angle = self.normalize_angle(cone_angle - heading)

            if abs(angle) > (math.pi / 2.0):
                continue

            if distance < min_distance:
                min_distance = distance
                result = cone

        return result

    def sort_cones(self, cones, closest_cone, heading):
        cone_chain = [closest_cone]
        cone_seen = {cones.index(closest_cone)}
        current_heading = heading

        while True:
            anchor_cone = cone_chain[-1]
            best_cone = None
            best_index = None
            best_distance = float("inf")
            best_dx = 0.0
            best_dy = 0.0

            for i, cone in enumerate(cones):
                if i in cone_seen:
                    continue

                dx = cone.x - anchor_cone.x
                dy = cone.y - anchor_cone.y
                distance_to_neighboring_cone = math.hypot(dx, dy)

                if distance_to_neighboring_cone > self.max_hop:
                    continue

                cone_angle = math.atan2(dy, dx)
                angle = self.normalize_angle(cone_angle - current_heading)
                if abs(angle) > (math.pi / 2.0):
                    continue

                if distance_to_neighboring_cone < best_distance:
                    best_distance = distance_to_neighboring_cone
                    best_cone = cone
                    best_index = i
                    best_dx = dx
                    best_dy = dy

            if best_cone is None:
                break

            cone_chain.append(best_cone)
            cone_seen.add(best_index)
            current_heading = math.atan2(best_dy, best_dx)

        return cone_chain

    def fit_boundary(self, chain):
        if len(chain) < 4:
            return None

        xcoord = []
        ycoord = []
        for cone in chain:
            if xcoord and math.hypot(cone.x - xcoord[-1], cone.y - ycoord[-1]) < 0.1:
                continue
            xcoord.append(cone.x)
            ycoord.append(cone.y)

        if len(xcoord) < 4:
            return None

        try:
            tck, u = splprep([xcoord, ycoord], s=self.spline_smoothing)
        except Exception as e:
            self.get_logger().warn(f"Spline fit failed: {e}")
            return None

        u_new = np.linspace(0.0, 1.0, self.num_samples)
        sx, sy = splev(u_new, tck)
        return list(zip(sx, sy))

    def build_centerline(self, left_points, right_points):
        centerline = []
        for (lx, ly), (rx, ry) in zip(left_points, right_points):
            centerline.append(((lx + rx) / 2.0, (ly + ry) / 2.0))
        return centerline

    def publish_path(self, centerline):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        for (x, y) in centerline:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def cones_callback(self, msg: Cones):
        """Callback for the cones subscriber."""
        if self.latest_odom is None:
            self.get_logger().warn("Odometry not received yet. Skipping.")
            return

        position = self.latest_odom.pose.pose.position
        heading = self.calculate_heading_from_quaternion(
            self.latest_odom.pose.pose.orientation
        )

        blue_cones = self.extract_cones_by_color(msg, "blue")
        yellow_cones = self.extract_cones_by_color(msg, "yellow")

        blue_seed = self.find_seed_cone(blue_cones, position, heading)
        yellow_seed = self.find_seed_cone(yellow_cones, position, heading)

        blue_chain = self.sort_cones(blue_cones, blue_seed, heading) if blue_seed else []
        yellow_chain = self.sort_cones(yellow_cones, yellow_seed, heading) if yellow_seed else []

        blue_boundary = self.fit_boundary(blue_chain)
        yellow_boundary = self.fit_boundary(yellow_chain)

        if blue_boundary is None or yellow_boundary is None:
            if self.latest_raw_path is not None:
                self.path_pub.publish(self.latest_raw_path)
                self.get_logger().warn("Boundary fit failed. Forwarding triangulator path.")
            else:
                self.get_logger().warn("Boundary fit failed and no triangulator path available.")
            return

        centerline = self.build_centerline(blue_boundary, yellow_boundary)
        self.publish_path(centerline)

        self.get_logger().info(
            f"blue={len(blue_cones)} yellow={len(yellow_cones)} "
            f"blue_chain={len(blue_chain)} yellow_chain={len(yellow_chain)} "
            f"centerline={len(centerline)}"
        )

    def calculate_heading_from_quaternion(self, quat: Quaternion) -> float:
        """Convert quaternion orientation into yaw heading in radians."""
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        """Wrap angle to the interval [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args: list[str] | None = None) -> None:
    """Start and spin the spline node."""
    rclpy.init(args=args)
    node = Spline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()