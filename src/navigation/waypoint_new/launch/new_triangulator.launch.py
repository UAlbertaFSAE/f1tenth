from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("waypoint_new"),
        "config",
        "triangulator_params.yaml",
    )

    triangulator_node = Node(
        package="waypoint_new",
        executable="new_triangulator",
        name="triangulator",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription(
        [
            triangulator_node,
        ]
    )
