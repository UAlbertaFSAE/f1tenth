"""Exercise the built controller over ROS: path replacement and speed tiers.

Run with an isolated ROS_DOMAIN_ID and the workspace environment sourced:
  python3 -m unittest discover -s src/navigation/pure_pursuit/test -v
"""

import signal
import subprocess
import time
import unittest

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from visualization_msgs.msg import Marker


class PathControllerTest(unittest.TestCase):
    def test_path_replacement_and_speed(self):
        rclpy.init()
        node = rclpy.create_node('controller_test')
        command = get_package_prefix('pure_pursuit') + '/lib/pure_pursuit/pure_pursuit'
        proc = subprocess.Popen([
            command, '--ros-args', '-r', '__node:=tested_controller',
            '-r', '/odom:=/test/odom', '-r', '/waypoints:=/test/path',
            '-r', '/drive:=/test/drive',
            '-r', '/lookahead_waypoint:=/test/lookahead',
            '-p', 'K_p:=0.6', '-p', 'waypoint_velocity:=6.0',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        paths = node.create_publisher(Path, '/test/path', 10)
        odoms = node.create_publisher(Odometry, '/test/odom', 10)
        drives, markers = [], []
        node.create_subscription(AckermannDriveStamped, '/test/drive', drives.append, 10)
        node.create_subscription(Marker, '/test/lookahead', markers.append, 10)
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose.orientation.w = 1.0

        def cycle(points):
            path = Path()
            path.header.frame_id = 'map'
            for x, y in points:
                pose = PoseStamped()
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                paths.publish(path)
                odoms.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(drives)
            return drives[-1].drive

        try:
            deadline = time.monotonic() + 5
            while paths.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertGreater(paths.get_subscription_count(), 0)
            self.assertAlmostEqual(cycle([(1, 0), (2, 0)]).speed, 6.0)
            self.assertAlmostEqual(cycle([(2, 1)]).speed, 2.52)
            self.assertAlmostEqual(cycle([(1, 1)]).speed, 1.98)
            # A new, shorter path entirely on the other side must replace old targets.
            self.assertLess(cycle([(2, -1)]).steering_angle, 0.0)
            self.assertAlmostEqual(markers[-1].pose.position.y, -1.0)
            client = node.create_client(SetParameters, '/tested_controller/set_parameters')
            self.assertTrue(client.wait_for_service(timeout_sec=2))
            future = client.call_async(SetParameters.Request(parameters=[Parameter('waypoint_velocity', value=4.0).to_parameter_msg()]))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            self.assertTrue(future.result().results[0].successful)
            # The existing parameter refresh timer runs every two seconds.
            deadline = time.monotonic() + 2.2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertAlmostEqual(cycle([(2, 0)]).speed, 4.0)
        finally:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
