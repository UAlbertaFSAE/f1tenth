from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # talker_params = [
    #     "v": 1.0,  # default value for speed
    #     "d": 0.5  # default value for steering angle
    # ]

    return LaunchDescription(
        [
            Node(
                package="lab1_pkg",
                executable="talker",
                name="talker",
                # parameters=[
                #     "v": 1.0,
                #     "d": 0.5,
                # ],
            ),
            Node(package="lab1_pkg", executable="relay", name="relay"),
        ]
    )
