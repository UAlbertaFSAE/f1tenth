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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_cmd(entry, log_suffix):
    pkg = entry.get("package")
    launch_file = entry.get("launch_file")
    executable = entry.get("executable")
    args = entry.get("args", [])

    if isinstance(args, (list, tuple)):
        args_str = " ".join(str(a) for a in args)
    else:
        args_str = str(args) if args else ""

    if launch_file:
        return f"ros2 launch {pkg} {launch_file} {args_str} {log_suffix}"
    if executable:
        return f"ros2 run {pkg} {executable} {args_str} {log_suffix}"

    return None


def _resolve_config_path(pkg_share_dir: str, config_value: str) -> Path:
    """Resolve a config path.

    - If `config_value` is absolute, use it.
    - Else, first try `${share}/config/<config_value>`.
    - Else, treat as a path relative to the current working directory.
    """
    if not config_value:
        config_value = "config.yaml"

    candidate = Path(config_value)
    if candidate.is_absolute():
        return candidate

    in_pkg_config_dir = Path(pkg_share_dir) / "config" / config_value
    if in_pkg_config_dir.exists():
        return in_pkg_config_dir

    # Fall back to CWD-relative path (useful for custom configs).
    return (Path.cwd() / config_value).resolve()


def _launch_setup(context, *args, **kwargs):
    pkg_share_dir = get_package_share_directory("launch_pkg")
    config_value = LaunchConfiguration("config").perform(context)
    cfg_path = _resolve_config_path(pkg_share_dir, config_value)

    if not cfg_path.exists():
        raise RuntimeError(
            "Config file not found: "
            f"{cfg_path}. Pass `config:=config.yaml` (from launch_pkg/config/) "
            "or an absolute path."
        )

    with open(cfg_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    recording_cfg = cfg.get("recording", {})
    packages_cfg = cfg.get("packages", [])
    static_transforms_cfg = cfg.get("static_transforms", [])
    rviz_cfg = cfg.get("rviz", {})
    rosbag_cfg = cfg.get("rosbag", {})

    save_recordings = recording_cfg.get("save", True)
    recordings_folder = recording_cfg.get("folder", "recordings")
    rosbag_enabled = rosbag_cfg.get("enabled", True)

    if save_recordings:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_log_dir = Path.cwd() / recordings_folder / timestamp
        run_log_dir.mkdir(parents=True, exist_ok=True)
        node_log_dir = run_log_dir / "node_logs"
        node_log_dir.mkdir(parents=True, exist_ok=True)
        bag_output_dir = run_log_dir / "rosbag"
    else:
        run_log_dir = Path.cwd()
        node_log_dir = None
        bag_output_dir = None

    rosbag_topics = rosbag_cfg.get("topics", [])

    launch_actions = [SetLaunchConfiguration("log_dir", str(run_log_dir))]

    # Iterate packages and create ExecuteProcess entries
    for idx, entry in enumerate(packages_cfg):
        name = entry.get("name") or f"pkg_{idx}"

        # prepare log file for this package
        pkg_log = node_log_dir / \
            f"{name}.txt" if node_log_dir is not None else None
        log_suffix = (
            f'2>&1 | tee -a "{pkg_log}"'
            if save_recordings and pkg_log is not None
            else ""
        )

        cmd = _build_cmd(entry, log_suffix)
        if not cmd:
            # skip invalid entries
            continue

        launch_actions.append(
            ExecuteProcess(
                cmd=["bash", "-lc", cmd],
                name=f"{name}_launch",
                output="screen",
            )
        )

    # static transforms
    for transform in static_transforms_cfg:
        translation = transform.get("translation", [0.0, 0.0, 0.0])
        rotation_rpy = transform.get("rotation_rpy", [0.0, 0.0, 0.0])
        parent_frame = transform.get("parent_frame", "map")
        child_frame = transform.get("child_frame", "base_link")

        launch_actions.append(
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

    # RViz
    rviz_config_value = rviz_cfg.get("config_file", "vizualizer.rviz")
    rviz_config_path = _resolve_config_path(pkg_share_dir, rviz_config_value)
    launch_actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(rviz_config_path)],
            output="screen",
        )
    )

    # rosbag
    if save_recordings and rosbag_enabled and rosbag_topics:
        launch_actions.insert(
            -1,
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o",
                     str(bag_output_dir), *rosbag_topics],
                name="rosbag_record",
                output="log",
            ),
        )

    return launch_actions


def generate_launch_description():
    """Generic launcher: iterates `packages` entries defined in config.yaml.

    Launch args:
      config: Which YAML config to load.
        - Default: config.yaml (from launch_pkg/config/)
        - You can also pass an absolute path.

    Expected YAML structure (top-level keys):
      recording: { save: true, folder: recordings }
      packages:
        - package: my_pkg
          launch_file: something.launch.py   # or `executable: bin_name`
          args: ["arg1:=value", "--flag"]
      static_transforms: [...]  # same format as other launch files
      rviz: { config_file: vizualizer.rviz }
      rosbag: { enabled: true, topics: [...] }
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="config.yaml",
                description=(
                    "YAML config file to load. If not an absolute path, it is resolved "
                    "relative to the launch_pkg/config/ directory."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
