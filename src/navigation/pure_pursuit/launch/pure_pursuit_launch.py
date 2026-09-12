"""Launch the controller with a package-relative or absolute tuning file."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def launch_controller(context):
    config = Path(LaunchConfiguration("config_file").perform(context))
    if not config.is_absolute():
        config = Path(get_package_share_directory("pure_pursuit")) / "config" / config
    return [
        Node(
            package="pure_pursuit",
            executable="pure_pursuit",
            name="pure_pursuit",
            parameters=[str(config)],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value="sim_config.yaml"),
            OpaqueFunction(function=launch_controller),
        ]
    )
