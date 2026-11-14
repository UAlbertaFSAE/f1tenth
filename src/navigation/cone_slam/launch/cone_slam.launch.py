"""Run SLAM without requiring a planner, controller, path or sensor launch."""

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_node(context: LaunchContext) -> list[Node]:
    """Apply explicit map and mode overrides to the package configuration."""
    overrides = {}
    for name in ("mode", "map_path", "output_map_path", "source_commit", "odom_topic"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    overrides["use_sim_time"] = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )
    return [
        Node(
            package="cone_slam",
            executable="cone_slam",
            name="cone_slam",
            output="screen",
            parameters=[LaunchConfiguration("config_file"), overrides],
            additional_env={"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"},
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare standalone mapping and localization launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("cone_slam"), "config", "cone_slam.yaml"]
                ),
            ),
            *[
                DeclareLaunchArgument(name, default_value="")
                for name in (
                    "mode",
                    "map_path",
                    "output_map_path",
                    "source_commit",
                    "odom_topic",
                )
            ],
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=launch_node),
        ]
    )
