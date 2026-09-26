from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    # 1. Declare the command-line arguments (with default values fallback)
    v_launch_arg = DeclareLaunchArgument(
        "v", default_value="0.0", description="Speed of the vehicle (m/s)"
    )

    d_launch_arg = DeclareLaunchArgument(
        "d", default_value="0.0", description="Steering angle of the vehicle (rad)"
    )

    return LaunchDescription(
        [
            # Include the declared arguments in the launch sequence
            v_launch_arg,
            d_launch_arg,
            # Spin up the odom_publisher node mapping parameters to launch configurations
            Node(
                package="onboarding",
                executable="odom_publisher",
                name="odom_publisher",
                output="screen",
                parameters=[
                    {"v": LaunchConfiguration("v"), "d": LaunchConfiguration("d")}
                ],
            ),
            # Spin up the odom_relay node
            Node(
                package="onboarding",
                executable="odom_relay",
                name="odom_relay",
                output="screen",
            ),
        ]
    )
