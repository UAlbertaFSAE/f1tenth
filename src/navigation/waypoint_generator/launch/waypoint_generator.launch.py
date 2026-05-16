import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("waypoint_generator"),
        "config",
        "triangulator_params.yaml",
    )

    triangulator_node = Node(
        package="waypoint_generator",
        executable="waypoint_generator",
        name="triangulator",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription(
        [
            triangulator_node,
        ]
    )
