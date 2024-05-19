from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("interpolation_count", default_value="5"),
            Node(
                package="detection_generator",
                namespace="detection_generator",
                executable="detection_generator",
                name="perception",
            ),
            Node(
                package="waypoint_triangulation",
                namespace="waypoint_triangulation",
                executable="triangulator",
                name="triangulator",
                parameters=[
                    {"interpolation_count": LaunchConfiguration("interpolation_count")},
                ],
            ),
            Node(
                package="pure_pursuit",
                namespace="pure_pursuit",
                executable="pure_pursuit",
                name="pursuit",
            ),
        ],
    )
