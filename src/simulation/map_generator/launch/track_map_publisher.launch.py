from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch the ground-truth track marker publisher with parameter file support."""
    config_file = LaunchConfiguration("config_file")

    default_config = PathJoinSubstitution(
        [FindPackageShare("map_generator"), "config", "config.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to track_map_publisher parameter YAML file",
            ),
            Node(
                package="map_generator",
                executable="track_map_publisher",
                name="track_map_publisher_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
