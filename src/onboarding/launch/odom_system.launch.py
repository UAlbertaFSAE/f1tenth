from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments for parameters v and d
    v_arg = DeclareLaunchArgument(
        'v',
        default_value='0.0',
        description='Speed parameter(odom_publisher)'
    )

    d_arg = DeclareLaunchArgument(
        'd',
        default_value='0.0',
        description='Steering angle(odom_publisher)'
    )

    # Launch the odom_publisher node
    odom_publisher_node = Node(
        package='onboarding',
        executable='odom_publisher',
        name='odom_publisher',
        parameters=[{
            'v': LaunchConfiguration('v'),
            'd': LaunchConfiguration('d')
        }]
    )

    # Launch the odom_relay node
    odom_relay_node = Node(
        package='onboarding',
        executable='odom_relay',
        name='odom_relay'
    )

    return LaunchDescription([
        v_arg,
        d_arg,
        odom_publisher_node,
        odom_relay_node
    ])
