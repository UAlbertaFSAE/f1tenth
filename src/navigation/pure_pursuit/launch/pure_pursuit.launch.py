from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    config_file_path = PathJoinSubstitution(
        [FindPackageShare("pure_pursuit"), "config", config_file]
    )

    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="pure_pursuit",
        name="pure_pursuit",
        parameters=[config_file_path],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="config.yaml",
            description="Pure Pursuit parameter file name or path.",
        ),
        pure_pursuit_node,
    ])
