# MIT License

# Copyright (c) 2026 Krupal Shah

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the autonomous system launch description from YAML configuration."""
    # config and args
    autonomous_config = os.path.join(
        get_package_share_directory("launch_pkg"), "config", "config.yaml"
    )
    with open(autonomous_config, encoding="utf-8") as config_file:
        autonomous_config_dict = yaml.safe_load(config_file) or {}

    zed_cfg = autonomous_config_dict.get("zed_wrapper", {})
    detection_cfg = autonomous_config_dict.get("detection", {})
    waypoint_cfg = autonomous_config_dict.get("waypoint", {})
    pure_pursuit_cfg = autonomous_config_dict.get("pure_pursuit", {})
    rviz_cfg = autonomous_config_dict.get("rviz", {})
    recording_cfg = autonomous_config_dict.get("recording", {})
    rosbag_cfg = autonomous_config_dict.get("rosbag", {})
    static_transforms_cfg = autonomous_config_dict.get("static_transforms", [])

    save_recordings = recording_cfg.get("save", True)
    recordings_folder = recording_cfg.get("folder", "recordings")
    rosbag_enabled = rosbag_cfg.get("enabled", True)

    if save_recordings:
        # Create a timestamped run folder (so logs/bags don't overwrite)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_log_dir = Path.cwd() / recordings_folder / timestamp
        run_log_dir.mkdir(parents=True, exist_ok=True)

        node_log_dir = run_log_dir / "node_logs"
        node_log_dir.mkdir(parents=True, exist_ok=True)
        bag_output_dir = run_log_dir / "rosbag"

        zed_launch_log = node_log_dir / "zed_camera_launch.txt"
        detection_launch_log = node_log_dir / "camera_detection_launch.txt"
        waypoint_launch_log = node_log_dir / "new_triangulator_launch.txt"
        pure_pursuit_log = node_log_dir / "pure_pursuit.txt"
    else:
        run_log_dir = Path.cwd()
        zed_launch_log = None
        detection_launch_log = None
        waypoint_launch_log = None
        pure_pursuit_log = None
        bag_output_dir = None

    zed_log_suffix = (
        f'2>&1 | tee -a "{zed_launch_log}"' if save_recordings else ""
    )
    detection_log_suffix = (
        f'2>&1 | tee -a "{detection_launch_log}"' if save_recordings else ""
    )
    waypoint_log_suffix = (
        f'2>&1 | tee -a "{waypoint_launch_log}"' if save_recordings else ""
    )
    pure_pursuit_log_suffix = (
        f'2>&1 | tee -a "{pure_pursuit_log}"' if save_recordings else ""
    )

    rosbag_topics = rosbag_cfg.get("topics", [])

    zed_cmd = (
        f"ros2 launch {zed_cfg.get('package', 'zed_wrapper')} {zed_cfg.get('launch_file', 'zed_camera.launch.py')} "
        f"camera_model:={zed_cfg.get('camera_model', 'zed2i')} "
        f"publish_tf:={str(zed_cfg.get('publish_tf', False)).lower()} "
        f"publish_map_tf:={str(zed_cfg.get('publish_map_tf', False)).lower()} "
        f"publish_urdf:={str(zed_cfg.get('publish_urdf', True)).lower()} "
        f"{zed_log_suffix}"
    )

    detection_cmd = (
        f"ros2 launch {detection_cfg.get('package', 'detection_camera')} "
        f"{detection_cfg.get('launch_file', 'camera_detection.launch.py')} "
        f"{detection_log_suffix}"
    )

    waypoint_cmd = (
        f"ros2 launch {waypoint_cfg.get('package', 'waypoint_new')} "
        f"{waypoint_cfg.get('launch_file', 'new_triangulator.launch.py')} "
        f"{waypoint_log_suffix}"
    )

    pure_pursuit_cmd = (
        f"ros2 run {pure_pursuit_cfg.get('package', 'pure_pursuit')} "
        f"{pure_pursuit_cfg.get('executable', 'pure_pursuit')} "
        f"{pure_pursuit_log_suffix}"
    )

    zed_launch = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            zed_cmd,
        ],
        name="zed_camera_launch",
        output="screen",
    )

    detection_launch = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            detection_cmd,
        ],
        name='camera_detection_launch',
        output='screen',
    )

    triangulator_launch = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            waypoint_cmd,
        ],
        name="new_triangulator_launch",
        output="screen",
    )

    waypoint_launch = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            waypoint_cmd,
        ],
        name='new_triangulator_launch',
        output='screen',
    )

    pure_pursuit = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            pure_pursuit_cmd,
        ],
        name="pure_pursuit",
        output="screen",
    )

    # RViz (loads your config from launch_pkg/config/visualizer.rviz)
    pkg_share = FindPackageShare("launch_pkg")
    rviz_config_path = PathJoinSubstitution(
        [pkg_share, "config", rviz_cfg.get("config_file", "visualizer.rviz")]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    rosbag_record = None
    if save_recordings and rosbag_enabled and rosbag_topics:
        rosbag_record = ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o",
                 str(bag_output_dir), *rosbag_topics],
            name="rosbag_record",
            output="log",
        )

    static_transform_processes = []
    for transform in static_transforms_cfg:
        translation = transform.get("translation", [0.0, 0.0, 0.0])
        rotation_rpy = transform.get("rotation_rpy", [0.0, 0.0, 0.0])
        parent_frame = transform.get("parent_frame", "map")
        child_frame = transform.get("child_frame", "base_link")

        static_transform_processes.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    str(translation[0]),
                    str(translation[1]),
                    str(translation[2]),
                    str(rotation_rpy[0]),
                    str(rotation_rpy[1]),
                    str(rotation_rpy[2]),
                    parent_frame,
                    child_frame,
                ],
                output="screen",
            )
        )

    launch_actions = [
        SetLaunchConfiguration("log_dir", str(run_log_dir)),
        zed_launch,
        detection_launch,
        *static_transform_processes,
        triangulator_launch,
        waypoint_launch,
        pure_pursuit,
        rviz_node,
    ]

    if rosbag_record is not None:
        launch_actions.insert(-1, rosbag_record)

    return LaunchDescription(launch_actions)
