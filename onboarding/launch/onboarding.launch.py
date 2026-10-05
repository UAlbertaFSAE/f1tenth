from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    v_arg = DeclareLaunchArgument(
        "v", default_value="0.0", description="Target velocity"
    )
    d_arg = DeclareLaunchArgument(
        "d", default_value="0.0", description="Target steering angle"
    )

    publisher_node = Node(
        package="onboarding",
        executable="odom_publisher",
        name="odom_publisher",
        parameters=[{"v": LaunchConfiguration("v"), "d": LaunchConfiguration("d")}],
    )

    relay_node = Node(
        package="onboarding",
        executable="odom_relay",
        name="odom_relay",
    )

    return LaunchDescription([v_arg, d_arg, publisher_node, relay_node])
