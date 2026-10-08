# MIT License
#
# Copyright (c) 2026 UAlbertaFSAE
# SPDX-License-Identifier: MIT

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_node(context: LaunchContext) -> list[Node]:
    """Override YAML values only when a launch argument is supplied."""
    overrides = {}
    for name in ("odom_topic", "frame_id"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    return [
        Node(
            package="path_planning",
            executable="path_planner",
            name="path_planner",
            output="screen",
            parameters=[LaunchConfiguration("config_file"), overrides],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Launch the planner with YAML configuration and optional overrides."""
    default_config = PathJoinSubstitution(
        [FindPackageShare("path_planning"), "config", "config.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("odom_topic", default_value=""),
            DeclareLaunchArgument("frame_id", default_value=""),
            OpaqueFunction(function=launch_node),
        ]
    )
