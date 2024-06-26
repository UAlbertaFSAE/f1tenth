from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
            ),
            Node(
                package="pure_pursuit",
                namespace="pure_pursuit",
                executable="pure_pursuit",
                name="pursuit",
            ),
        ]
    )
