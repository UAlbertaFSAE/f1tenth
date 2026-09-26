# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the triangulator node."""
    config = os.path.join(
        get_package_share_directory("path_planning"),
        "config",
        "config.yaml",
    )

    triangulator_node = Node(
        package="path_planning",
        executable="triangulator_node",
        name="triangulator_node",
        output="screen",
        parameters=[config, {"odom_topic": LaunchConfiguration("odom_topic")}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            triangulator_node,
        ]
    )
