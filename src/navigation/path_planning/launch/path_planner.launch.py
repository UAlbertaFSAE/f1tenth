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
    lidar_corridor = LaunchConfiguration("lidar_corridor_enabled").perform(context)
    if lidar_corridor:
        if lidar_corridor.lower() not in ("true", "false"):
            raise ValueError("lidar_corridor_enabled must be true or false")
        overrides["lidar_corridor_enabled"] = lidar_corridor.lower() == "true"
    for name in ("odom_topic", "frame_id", "cones_topic"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    for name in ("delaunay_max_edge_m", "track_width_m", "hold_timeout_s"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = float(value)
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
            DeclareLaunchArgument("cones_topic", default_value=""),
            DeclareLaunchArgument("hold_timeout_s", default_value=""),
            DeclareLaunchArgument("lidar_corridor_enabled", default_value=""),
            DeclareLaunchArgument("delaunay_max_edge_m", default_value=""),
            DeclareLaunchArgument("track_width_m", default_value=""),
            OpaqueFunction(function=launch_node),
        ]
    )
