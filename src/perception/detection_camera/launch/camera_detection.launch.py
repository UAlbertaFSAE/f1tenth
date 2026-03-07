from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch camera detection nodes along with RVIZ and triangulator."""
    pkg_share = FindPackageShare("detection_camera")

    config_file_path = PathJoinSubstitution(
        [pkg_share, "config", "camera_detection.yaml"]
    )

    rviz_config_path = PathJoinSubstitution([pkg_share, "config", "detection_viz.rviz"])

    detection_camera_node = Node(
        package="detection_camera",
        executable="detection_camera",
        name="detection_camera",
        output="screen",
        parameters=[config_file_path],
    )

    # Your triangulator Python node (installed by waypoint_triangulation CMake)
    triangulator_node = Node(
        package="waypoint_triangulation",
        executable="new_triangulator.py",
        name="triangulator",
        output="screen",
        parameters=[{
            # MUST match the detection node output topic:
            "cones_topic": "/cone_positions",

            # MUST be a real odom topic in your system (ZED or simulator):
            "odom_topic": "/ego_racecar/odom",

            "waypoint_topic": "/waypoints",
            "path_topic": "/planned_path",
            "lookahead_distance": 10.0,
            "num_waypoints": 5,
            "min_track_width": 2.0,
            "max_track_width": 6.0,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="detection_rviz",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription([
        detection_camera_node,
        triangulator_node,
        rviz_node,
    ])
