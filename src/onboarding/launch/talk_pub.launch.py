from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='onboarding',
            namespace='odm_publisher',
            executable='odm_publisher',
            name='sim'
        ),
        Node(
            package='onboarding',
            namespace='odm_relay',
            executable='odm_relay',
            name='sim'
        ),
    ])
