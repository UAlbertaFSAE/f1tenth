from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    config_file_path = PathJoinSubstitution(
        [FindPackageShare("pure_pursuit"), "config", "config.yaml"]
    )

    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="pure_pursuit",
        name="pure_pursuit",
        parameters=[config_file_path],
    )

    return LaunchDescription([
        pure_pursuit_node,
    ])
