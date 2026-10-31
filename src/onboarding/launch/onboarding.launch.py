
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Get values supplied through ros2 launch
    v = LaunchConfiguration('v')
    d = LaunchConfiguration('d')

    return LaunchDescription([

        # Define launch arguments and their defaults
        DeclareLaunchArgument('v', default_value='0.0'),
        DeclareLaunchArgument('d', default_value='0.0'),

        # Start the original publisher node
        Node(
            package='onboarding',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            parameters=[{
                'v': ParameterValue(v, value_type=float),
                'd': ParameterValue(d, value_type=float),
            }]
        ),

        # Start the relay node
        Node(
            package='onboarding',
            executable='odom_relay',
            name='odom_relay',
            output='screen'
        ),
    ])
