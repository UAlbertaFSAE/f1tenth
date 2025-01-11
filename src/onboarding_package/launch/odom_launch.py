from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    declare_v = DeclareLaunchArgument(
        "v", default_value="1.0", description="Speed parameter"
    )
    declare_d = DeclareLaunchArgument(
        "d", default_value="1.0", description="Steering angle parameter"
    )


    odom_publisher_node = Node(
        package="onboarding_package",
        executable="odom_publisher",
        name="odom_publisher",
        parameters=[
            {
                "v": LaunchConfiguration("v"),
                "d": LaunchConfiguration("d"),
            }
        ],
    )
    odom_relay_node = Node(
        package="onboarding_package",
        executable="odom_relay",
        name ="odom_relay",
    )

    return LaunchDescription(
        [
            declare_v,
            declare_d,
            odom_publisher_node,
            odom_relay_node,
        ]
    )
