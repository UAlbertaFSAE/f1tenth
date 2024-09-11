from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates launch description for onboarding task.

    The speed and steering angle parameters for the odom_publisher node can be
    set through the command line when launching with v:=<value> and d:=<value>.
    They default to 0.0 each.

    Returns:
        LaunchDescription: describes how the onboarding system should be launched
    """
    speed = DeclareLaunchArgument("v", default_value="0.0", description="speed")
    steering_angle = DeclareLaunchArgument(
        "d", default_value="0.0", description="steering angle"
    )

    return LaunchDescription(
        [
            speed,
            steering_angle,
            Node(
                package="onboarding",
                executable="odom_publisher",
                parameters=[
                    {"v": LaunchConfiguration("v"), "d": LaunchConfiguration("d")}
                ],
            ),
            Node(
                package="onboarding",
                executable="odom_relay",
            ),
        ]
    )
