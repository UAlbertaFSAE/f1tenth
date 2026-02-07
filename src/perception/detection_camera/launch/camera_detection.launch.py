from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch camera detection nodes along with RVIZ."""
    # Get package share directory
    pkg_share = FindPackageShare("detection_camera")

    # Path to config file
    config_file_path = PathJoinSubstitution(
        [pkg_share, "config", "camera_detection.yaml"]
    )

    # Path to RViz config file
    rviz_config_path = PathJoinSubstitution([pkg_share, "config", "detection_viz.rviz"])

    # Detection camera node - loads all parameters from YAML config file
    detection_camera_node = Node(
        package="detection_camera",
        executable="detection_camera",
        name="detection_camera",
        output="screen",
        parameters=[config_file_path],
    )

    # RViz2 node (reads visualize parameter from config file)
    # Note: We can't conditionally launch based on YAML param easily,
    # so RViz will launch by default. You can disable by commenting out rviz_node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="detection_rviz",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription([detection_camera_node, rviz_node])
