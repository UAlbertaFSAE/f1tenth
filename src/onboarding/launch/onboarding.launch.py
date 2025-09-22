from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:  # noqa: D103
    v_arg = DeclareLaunchArgument(
        "v",
        default_value="0.0",
        description="Linear velocity parameter"
    )

    d_arg = DeclareLaunchArgument(
        "d",
        default_value="0.0",
        description="Distance parameter"
    )

    publisher_node = Node(
        package="onboarding",
        executable="odom_publisher",
        name="publisher",
        parameters=[{
            "v": LaunchConfiguration("v"),
            "d": LaunchConfiguration("d")
        }]
    )

    relay_node = Node(
        package="onboarding",
        executable="odom_relay",
        name="relay",
        remappings=[
            ('odom_sample', 'odom_sample'),
            ('drive', 'drive')
        ]
    )

    return LaunchDescription([
        v_arg,
        d_arg,
        publisher_node,
        relay_node
    ])
