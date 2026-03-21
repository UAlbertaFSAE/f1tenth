from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("v", default_value="0.0"),
            DeclareLaunchArgument("d", default_value="0.0"),
            # Start the Publisher Node
            Node(
                package="onboarding",
                executable="odom_publisher",
                name="odom_publisher_node",
                parameters=[
                    {"v": LaunchConfiguration(
                        "v"), "d": LaunchConfiguration("d")}
                ],
            ),
            # Start the Relay Node
            Node(
                package="onboarding",
                executable="odom_relay",
                name="odom_relay_node",
            ),
        ]
    )
