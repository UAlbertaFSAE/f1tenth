#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# create a launch file in a directory called launch just inside your package directory
# add the launch folder to your package configuration (CMakeLists.txt if using C++, setup.py if using Python)
# the ros2 docs on launch files may be of use here, spin up your odom_publisher and your odom_relay nodes
# inside this launch file in such a way that you can set the v and d parameters in the command line
# when running the ros2 launch ... command given below


def generate_launch_description() -> None:  # ROS2 looks for this function name
    speed = DeclareLaunchArgument(
        "v", default_value="0.0", description="speed"
    )  # setup args for command line
    steering = DeclareLaunchArgument("d", default_value="0.0", description="steering")

    publisher = Node(  # odom_publisher node
        package="onboarding",
        executable="odom_publisher",  # name from setup.py
        name="odom_publisher",
        parameters=[
            {
                "v": LaunchConfiguration(
                    "v"
                ),  # key matches w self.declare_parameter("v")
                "d": LaunchConfiguration("d"),  # key ~ d
            }
        ],
    )

    relay = Node(  # odom_relay node
        package="onboarding",
        executable="odom_relay",  # setup.py
        name="odom_relay",
    )

    # spin
    return LaunchDescription([speed, steering, publisher, relay])
