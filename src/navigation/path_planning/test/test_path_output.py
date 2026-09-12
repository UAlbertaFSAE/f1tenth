"""Regression checks for the Python planner's complete-path controller interface."""

from collections.abc import Iterator
from unittest.mock import Mock

import pytest
import rclpy
from nav_msgs.msg import Odometry, Path
from path_planning.triangulator import Triangulator
from rc_interfaces.msg import Cone, Cones


@pytest.fixture  # type: ignore[untyped-decorator]
def planner() -> Iterator[Triangulator]:
    """Create a planner with captured outputs and a known world pose."""
    rclpy.init()
    node = Triangulator()
    node.controller_path_pub = Mock()
    node.path_pub = Mock()
    odom = Odometry()
    odom.header.frame_id = "map"
    odom.pose.pose.orientation.w = 1.0
    node.odom_callback(odom)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def gates(xs: list[int], colors: tuple[str, str] = ("blue", "yellow")) -> Cones:
    """Create paired cones along the x axis."""
    return Cones(
        cones=[
            Cone(x=float(x), y=y, color=color)
            for x in xs
            for y, color in zip((1.0, -1.0), colors, strict=True)
        ]
    )


def test_complete_path_and_replacement(planner: Triangulator) -> None:
    """Every update contains the current gates, with no previous path points."""
    planner.cones_callback(gates([1, 2, 3]))
    first = planner.controller_path_pub.publish.call_args.args[0]
    assert isinstance(first, Path)
    assert [p.pose.position.x for p in first.poses] == [1.0, 2.0, 3.0]
    assert first.header.frame_id == "map"
    assert all(p.header == first.header for p in first.poses)
    assert planner.path_pub.publish.call_args.args[0] == first
    planner.cones_callback(gates([4]))
    second = planner.controller_path_pub.publish.call_args.args[0]
    assert [p.pose.position.x for p in second.poses] == [4.0]


def test_start_gate_uses_path(planner: Triangulator) -> None:
    """Start-gate targets use the same message contract as normal gates."""
    planner.use_start_gate = True
    planner.cones_callback(gates([2], ("orange", "orange")))
    path = planner.controller_path_pub.publish.call_args.args[0]
    assert isinstance(path, Path)
    assert len(path.poses) == 1
    assert path.poses[0].pose.position.x == 2.0
    assert path.header.frame_id == "map"
