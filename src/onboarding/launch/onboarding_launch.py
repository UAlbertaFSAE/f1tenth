from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="onboarding",
                executable="Odom_publisher",
                name = "odom_publisher" ,
            ),
            Node(
                package="onboarding",
                executable="Odom_relay",
                name = "odom_relay",
            ),
        ]
    )
