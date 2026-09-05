from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch camera detection nodes along with RVIZ and triangulator."""
    pkg_share = FindPackageShare("camera_detection")

    config_file_path = PathJoinSubstitution(
        [pkg_share, "config", "camera_detection.yaml"]
    )

    rviz_config_path = PathJoinSubstitution([pkg_share, "config", "detection_viz.rviz"])

    camera_detection_node = Node(
        package="camera_detection",
        executable="camera_detection",
        name="camera_detection",
        output="screen",
        parameters=[config_file_path],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="detection_rviz",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription(
        [
            camera_detection_node,
            rviz_node,
        ]
    )
