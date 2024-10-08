from launch import LaunchDescription
from launch_ros.actions import Node


# TODO: get range_min and max from zed2i yaml in perception zed_wrapper config folder
def generate_launch_description() -> LaunchDescription:
    """Generates the launch description for launching the safety node.

    Returns:
        LaunchDescription: Launch description object containing a few nodes
    """
    return LaunchDescription(
        [
            Node(
                package="depthimage_to_laserscan",
                executable="depthimage_to_laserscan_node",
                name="depthimage_to_laserscan",
                remappings=[
                    ("/depth", "/zed/zed_node/depth/depth_registered"),
                    ("/depth_camera_info", "zed/zed_node/depth/camera_info"),
                ],
                parameters=[
                    {"range_min": 0.2},
                    {"range_max": 20.0},
                    {"scan_height": 3},
                ],
            ),
            Node(
                package="safety_node",
                executable="safety_node",
                name="safety_node",
            ),
        ]
    )
