# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""Bring up the dashboard's WebSocket bridge and the page that talks to it.

Three things come up, all driven by ``config/dashboard.yaml``:

* one ``topic_tools`` throttle per configured topic, republishing it at a fixed
  rate on ``/dashboard/<name>``. They share a single component container, so
  the whole set is one process on the Jetson. They are lazy: until a browser
  subscribes, they do not subscribe to the stack either.
* ``rosbridge_websocket``, restricted to ``/dashboard/*``. rosbridge's topic
  glob governs publishing as well as subscribing, so exposing the real topics
  would let a browser publish a path straight into the controller. Exposing
  only the throttled copies keeps the stack read-only from the browser.
* a static HTTP server for ``web/``, so the page loads from the car with no
  internet connection and no ROS install on the viewing device.

Usage::

    ros2 launch dashboard dashboard.launch.py
    ros2 launch dashboard dashboard.launch.py port:=9091 http_port:=8001
"""

from __future__ import annotations

import json
import shutil
import tempfile
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

PACKAGE_NAME = "dashboard"

#: Namespace every bridged topic is republished under, and the only topics
#: rosbridge will touch.
DASHBOARD_NAMESPACE = "/dashboard"


def _load_config(config_path: str) -> dict[str, Any]:
    """Read and minimally validate the dashboard config."""
    with Path(config_path).open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    for entry in config.get("topics", []):
        missing = {"name", "source", "type", "rate"} - entry.keys()
        if missing:
            raise ValueError(
                f"Dashboard topic entry {entry} in {config_path} is missing "
                f"{sorted(missing)}"
            )
    return config


def _throttle_container(topics: list[dict[str, Any]]) -> ComposableNodeContainer:
    """Build one container holding a lazy throttle per configured topic."""
    nodes = [
        ComposableNode(
            package="topic_tools",
            plugin="topic_tools::ThrottleNode",
            name=f"dashboard_throttle_{index}_{entry['name']}",
            parameters=[
                {
                    "input_topic": entry["source"],
                    "output_topic": f"{DASHBOARD_NAMESPACE}/{entry['name']}",
                    "throttle_type": "messages",
                    "msgs_per_sec": float(entry["rate"]),
                    "lazy": True,
                }
            ],
        )
        for index, entry in enumerate(topics)
    ]
    return ComposableNodeContainer(
        name="dashboard_throttles",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )


def _web_root(topics: list[dict[str, Any]], port: int) -> Path:
    """Assemble the served directory: the page plus a generated topics.json.

    The page learns which topics exist from ``topics.json`` rather than keeping
    its own list, so ``dashboard.yaml`` stays the only place topics are named.
    It is written to a temporary directory because the installed share
    directory is a symlink back into the source tree under --symlink-install.
    """
    web_root = Path(tempfile.mkdtemp(prefix="dashboard_web_"))
    share_web = Path(get_package_share_directory(PACKAGE_NAME)) / "web"
    for source in share_web.iterdir():
        (web_root / source.name).symlink_to(source.resolve())

    # Several sources can feed one dashboard topic (/odom on the car,
    # /ego_racecar/odom in sim); the page only needs each output once.
    served: dict[str, dict[str, Any]] = {}
    for entry in topics:
        topic = f"{DASHBOARD_NAMESPACE}/{entry['name']}"
        served.setdefault(
            topic, {"topic": topic, "type": entry["type"], "sources": []}
        )["sources"].append(entry["source"])

    (web_root / "topics.json").write_text(
        json.dumps({"port": port, "topics": list(served.values())}, indent=2),
        encoding="utf-8",
    )
    return web_root


def _launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Any]:
    """Resolve the launch arguments and expand the config into actions."""
    del args, kwargs

    config = _load_config(LaunchConfiguration("config_file").perform(context))
    port = int(LaunchConfiguration("port").perform(context) or config.get("port", 9090))
    http_port = int(
        LaunchConfiguration("http_port").perform(context)
        or config.get("http_port", 8000)
    )
    topics = config.get("topics", [])
    web_root = _web_root(topics, port)

    return [
        _throttle_container(topics),
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[
                {
                    "port": port,
                    # Every interface, so a phone on the car's network can connect.
                    "address": "",
                    "topics_glob": f"['{DASHBOARD_NAMESPACE}/*']",
                    "services_glob": "[]",
                    "actions_glob": "[]",
                }
            ],
        ),
        ExecuteProcess(
            cmd=[
                "python3",
                "-m",
                "http.server",
                str(http_port),
                "--directory",
                str(web_root),
            ],
            name="dashboard_http",
            output="log",
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda *_: shutil.rmtree(web_root, ignore_errors=True)
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the launch arguments and defer the rest to ``_launch_setup``."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(
                    Path(get_package_share_directory(PACKAGE_NAME))
                    / "config"
                    / "dashboard.yaml"
                ),
                description="Path to the dashboard config YAML.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="",
                description="rosbridge WebSocket port. Empty uses the config value.",
            ),
            DeclareLaunchArgument(
                "http_port",
                default_value="",
                description="Web page port. Empty uses the config value.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
