# Copyright (c) 2026, Your Name. Subject to the MIT license.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start odom_publisher and odom_relay, passing v and d to the publisher."""
    v_arg = DeclareLaunchArgument("v", default_value="0.0", description="Speed in m/s.")
    d_arg = DeclareLaunchArgument(
        "d", default_value="0.0", description="Steering angle in radians."
    )

    odom_publisher = Node(
        package="onboarding",
        executable="odom_publisher",
        name="odom_publisher",
        namespace="onboarding",
        output="screen",
        parameters=[
            {
                "v": ParameterValue(LaunchConfiguration("v"), value_type=float),
                "d": ParameterValue(LaunchConfiguration("d"), value_type=float),
            }
        ],
    )

    odom_relay = Node(
        package="onboarding",
        executable="odom_relay",
        name="odom_relay",
        namespace="onboarding",
        output="screen",
    )

    return LaunchDescription([v_arg, d_arg, odom_publisher, odom_relay])
