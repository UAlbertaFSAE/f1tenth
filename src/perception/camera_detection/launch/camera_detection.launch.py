# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _model_file() -> str:
    """Return the weights to load, preferring a TensorRT engine when one exists.

    An engine is several times faster on the Jetson but is built for one GPU and
    one TensorRT version, so it is exported locally rather than committed. Fall
    back to the .pt weights that always ship with the package.
    """
    models_dir = os.path.join(get_package_share_directory("camera_detection"), "models")
    engine = os.path.join(models_dir, "model.engine")
    return engine if os.path.isfile(engine) else os.path.join(models_dir, "model.pt")


def generate_launch_description() -> LaunchDescription:
    """Launch camera detection nodes along with RVIZ and triangulator."""
    pkg_share = FindPackageShare("camera_detection")

    config_file_path = PathJoinSubstitution(
        [pkg_share, "config", "camera_detection.yaml"]
    )

    rviz_config_path = PathJoinSubstitution([pkg_share, "config", "detection_viz.rviz"])

    # The weights ship with the package, so the paths are resolved from the share
    # directory rather than written into the config as an absolute path that is
    # only correct on one machine.
    model_overrides = {
        "model_file": _model_file(),
        "classes_file": PathJoinSubstitution([pkg_share, "models", "classes.txt"]),
    }

    camera_detection_node = Node(
        package="camera_detection",
        executable="camera_detection",
        name="camera_detection",
        output="screen",
        parameters=[config_file_path, model_overrides],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="detection_rviz",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription(
        [
            camera_detection_node,
            rviz_node,
        ]
    )
