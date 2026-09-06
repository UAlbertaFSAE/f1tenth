# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""Workspace-level launch entry point for the F1TENTH stack.

One launch file drives the whole car. What actually comes up is decided by a
YAML config in this package's ``config/`` directory, selected with the
``config`` launch argument::

    ros2 launch launch_pkg fsae.launch.py                       # config.yaml
    ros2 launch launch_pkg fsae.launch.py config:=sim_config.yaml

The config is read at launch time (not at import time) inside an
``OpaqueFunction``, because the value of a launch argument is not known until
then. Every component is opt-in via ``launch:`` in that file, so a machine
without a ZED or a LiDAR runs the same launch file with a different config
rather than a different launch file.
"""

from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "launch_pkg"

#: Topics recorded when ``launch.rosbag`` is enabled and the config does not
#: override ``rosbag.topics``.
DEFAULT_ROSBAG_TOPICS = [
    "/clicked_point",
    "/cone_positions",
    "/current_waypoint",
    "/detection_visualization/depth",
    "/detection_visualization/detections",
    "/diagnostics",
    "/drive",
    "/lookahead_waypoint",
    "/odom",
    "/parameter_events",
    "/planned_path",
    "/rosout",
    "/tf",
    "/tf_static",
    "/waypoints",
]


def _load_config(config_name: str) -> dict[str, Any]:
    """Read ``config_name`` from this package's share/config directory."""
    config_path = (
        Path(get_package_share_directory(PACKAGE_NAME)) / "config" / config_name
    )
    if not config_path.is_file():
        raise FileNotFoundError(
            f"No such launch config: {config_path}. "
            f"Available: {sorted(p.name for p in config_path.parent.glob('*.yaml'))}"
        )
    with config_path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _make_run_dir() -> tuple[Path, Path]:
    """Create a timestamped run directory so logs and bags never overwrite."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path.cwd() / "recordings" / timestamp
    node_log_dir = run_dir / "node_logs"
    node_log_dir.mkdir(parents=True, exist_ok=True)
    return run_dir, node_log_dir


def _static_transform_nodes(transforms: list[dict[str, Any]]) -> list[Node]:
    """Build one ``static_transform_publisher`` per entry in ``transforms``."""
    nodes = []
    for index, transform in enumerate(transforms):
        xyz = [str(value) for value in transform.get("xyz", [0.0, 0.0, 0.0])]
        rpy = [str(value) for value in transform.get("rpy", [0.0, 0.0, 0.0])]
        parent = transform["parent"]
        child = transform["child"]
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_tf_{index}_{parent}_to_{child}",
                arguments=[*xyz, *rpy, parent, child],
                output="screen",
            )
        )
    return nodes


def _optional_package_launch(
    package: str,
    launch_file: str,
    log_path: Path,
    extra_args: dict[str, Any] | None = None,
) -> ExecuteProcess | None:
    """Launch a package that may not be built in this workspace.

    ``zed_wrapper`` and ``livox_ros_driver2`` are in the Makefile's
    ``PACKAGES_IGNORE`` default, so on a machine that ran a plain ``make build``
    they are genuinely absent. Returning ``None`` (with a warning) is better
    than an import-time crash that takes the whole stack down with it.
    """
    try:
        get_package_share_directory(package)
    except PackageNotFoundError:
        print(
            f"[fsae.launch.py] '{package}' is enabled in the config but is not "
            f"built in this workspace -- skipping it. Build it with "
            f"`make package {package}`."
        )
        return None

    args = " ".join(f"{key}:={value}" for key, value in (extra_args or {}).items())
    command = f"ros2 launch {package} {launch_file} {args}".strip()
    return ExecuteProcess(
        cmd=["bash", "-lc", f'{command} 2>&1 | tee -a "{log_path}"'],
        name=f"{package}_launch",
        output="screen",
    )


def _launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Any]:
    """Resolve the config argument and expand it into launch actions."""
    del args, kwargs

    config = _load_config(LaunchConfiguration("config").perform(context))
    enabled = config.get("launch", {})
    run_dir, node_log_dir = _make_run_dir()

    actions: list[Any] = []

    if enabled.get("zed_camera", False):
        zed = config.get("zed", {})
        action = _optional_package_launch(
            "zed_wrapper",
            "zed_camera.launch.py",
            node_log_dir / "zed_camera.txt",
            {
                "camera_model": zed.get("camera_model", "zed2i"),
                # We publish base_link -> zed_camera_link ourselves (see
                # static_transforms), so the wrapper must not also publish it.
                "publish_tf": str(zed.get("publish_tf", False)).lower(),
                "publish_map_tf": str(zed.get("publish_map_tf", False)).lower(),
                "publish_urdf": str(zed.get("publish_urdf", True)).lower(),
            },
        )
        if action is not None:
            actions.append(action)

    if enabled.get("lidar_driver", False):
        lidar = config.get("lidar", {})
        action = _optional_package_launch(
            "livox_ros_driver2",
            lidar.get("launch_file", "msg_MID360_launch.py"),
            node_log_dir / "livox_driver.txt",
        )
        if action is not None:
            actions.append(action)

    actions.extend(_static_transform_nodes(config.get("static_transforms", [])))

    if enabled.get("camera_detection", False):
        actions.append(
            Node(
                package="camera_detection",
                executable="camera_detection",
                name="camera_detection",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("camera_detection"),
                        "config",
                        "camera_detection.yaml",
                    ),
                    # The weights ship with the package; resolving them here keeps
                    # an absolute path off one person's machine out of the config.
                    {
                        "model_file": os.path.join(
                            get_package_share_directory("camera_detection"),
                            "models",
                            "model.pt",
                        ),
                        "classes_file": os.path.join(
                            get_package_share_directory("camera_detection"),
                            "models",
                            "classes.txt",
                        ),
                    },
                ],
            )
        )

    if enabled.get("lidar_cone_filtering", False):
        actions.append(
            Node(
                package="lidar_cone_filtering",
                executable="lidar_filtering_o3d",
                name="lidar_cone_filtering",
                output="screen",
            )
        )

    if enabled.get("path_planning", False):
        actions.append(
            Node(
                package="path_planning",
                executable="triangulator_node",
                name="triangulator_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("path_planning"),
                        "config",
                        "config.yaml",
                    )
                ],
            )
        )

    if enabled.get("pure_pursuit", False):
        # pure_pursuit is launched as-is; its own config handling is known to be
        # broken (#109 leaves it explicitly out of scope) and is not fixed here.
        actions.append(
            Node(
                package="pure_pursuit",
                executable="pure_pursuit",
                name="pure_pursuit",
                output="screen",
            )
        )

    if enabled.get("visualizer", False):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="visualizer",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory(PACKAGE_NAME),
                        "config",
                        "visualizer.rviz",
                    ),
                ],
                output="screen",
            )
        )

    if enabled.get("rosbag", False):
        topics = config.get("rosbag", {}).get("topics", DEFAULT_ROSBAG_TOPICS)
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", str(run_dir / "rosbag"), *topics],
                name="rosbag_record",
                output="log",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Declare the ``config`` argument and defer the rest to ``_launch_setup``."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="config.yaml",
                description=(
                    "Name of the YAML file in launch_pkg/config that decides "
                    "which components come up."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
