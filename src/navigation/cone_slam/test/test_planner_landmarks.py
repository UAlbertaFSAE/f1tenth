"""Check the planner's identity/color/covariance transport independently of search."""

from time import monotonic
from types import SimpleNamespace

import numpy as np
import rclpy
from cone_slam.mapper import COLORS, Landmark
from cone_slam.node import SlamNode
from nav_msgs.msg import Odometry
from rclpy.time import Time


def test_planner_metadata_rotates_covariance_preserves_color_and_clears_stale():
    """Keep measured labels and unknowns while transforming only geometry."""
    rclpy.init(
        args=["--ros-args", "-p", "use_sim_time:=true", "-p", "publish_tf:=false"]
    )
    node = SlamNode()
    node.close()
    outputs = []
    node.local_map_pub = SimpleNamespace(publish=outputs.append)
    try:
        node.get_clock().set_ros_time_override(Time(seconds=10))
        odom = Odometry()
        odom.header.stamp = Time(seconds=10).to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        node.on_odom(odom)
        colored = Landmark(7, state="confirmed")
        colored.colors[COLORS.index("blue")] = 3
        unknown = Landmark(8, state="confirmed")
        node.snapshot = dict(
            map_id="test",
            revision=1,
            localized=True,
            tracking_ready=True,
            reason="",
            last_observation=10.0,
            correction=np.array([5.0, 2.0, np.pi / 2]),
            pose_cov=np.eye(3),
            landmarks=[
                (colored, np.array([5.0, 4.0]), np.diag([0.01, 0.04])),
                (unknown, np.array([5.0, 6.0]), np.eye(2) * 0.02),
            ],
            optimization_ms=1.0,
            keyframes=3,
            factors=6,
            closures=0,
            closure_pending=False,
        )
        node.last_worker_wall = monotonic()
        node.publish()
        assert outputs[-1].header.frame_id == "odom"
        rows = outputs[-1].landmarks
        assert [row.id for row in rows] == [7, 8]
        assert [row.color for row in rows] == ["blue", ""]
        np.testing.assert_allclose(
            [rows[0].position.x, rows[0].position.y], [2, 0], atol=1e-10
        )
        np.testing.assert_allclose(
            np.array(rows[0].covariance).reshape(2, 2),
            np.diag([0.04, 0.01]),
            atol=1e-10,
        )
        assert unknown.color == "" and colored.color == "blue"
        node.get_clock().set_ros_time_override(Time(seconds=12))
        node.publish()
        assert not outputs[-1].landmarks
    finally:
        node.destroy_node()
        rclpy.shutdown()
