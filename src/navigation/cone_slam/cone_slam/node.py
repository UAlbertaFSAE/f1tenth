"""ROS transport around a single bounded SLAM worker; no planner dependency."""

import hashlib
import json
import math
import queue
import threading
from copy import deepcopy
from dataclasses import fields
from pathlib import Path
from time import monotonic
from typing import Any, cast

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rc_interfaces.msg import (
    Cone,
    ConeLandmark,
    ConeMap,
    ConeObservations,
    Cones,
    SlamStatus,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .geometry import OdomBuffer, OdomSample, compose, covariance, inverse, transform
from .mapper import Mapper, Observation, Settings
from .persistence import load, save


def seconds(stamp: Any) -> float:
    """Convert a ROS timestamp to seconds."""
    return float(stamp.sec + stamp.nanosec * 1e-9)


def quaternion_matrix(q: Any) -> np.ndarray:
    """Normalize a finite quaternion and return its rotation matrix."""
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError("invalid_quaternion")
    x, y, z, w = values / norm
    return cast(
        np.ndarray,
        np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        ),
    )


class SlamNode(Node):
    """Publish map, pose, quality and a local planner view independently."""

    def __init__(self) -> None:
        """Configure transport and start the single bounded graph worker."""
        super().__init__("cone_slam")
        defaults = Settings()
        kwargs = {
            f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
            for f in fields(defaults)
        }
        self.settings = Settings(**kwargs)
        self.mode = self.declare_parameter("mode", "mapping").value
        self.map_path = self.declare_parameter("map_path", "").value
        self.output_path = self.declare_parameter("output_map_path", "").value
        self.source_commit = self.declare_parameter("source_commit", "unknown").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        self.mapper = (
            load(self.map_path, self.settings, self.mode)
            if self.map_path
            else Mapper(self.settings, self.mode)
        )
        if self.mode != "mapping" and not self.map_path:
            raise ValueError("saved_map_required")
        if (
            self.map_path
            and self.output_path
            and Path(self.map_path).resolve() == Path(self.output_path).resolve()
        ):
            raise ValueError("output_must_not_overwrite_reference")
        self.buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.odom = OdomBuffer()
        self.lock = threading.Lock()
        self.jobs: queue.Queue[tuple[str, Any]] = queue.Queue(maxsize=32)
        self.stopping = threading.Event()
        self.fault = ""
        self.dropped = 0
        self.snapshot: dict[str, Any] = {}
        self.last_clock: float | None = None
        self.last_odom_wall = -float("inf")
        self.last_odom: Odometry | None = None
        self.last_worker_wall = monotonic()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos)
        for sensor in ("camera", "lidar"):
            topic = self.declare_parameter(
                f"{sensor}_topic", f"/perception/{sensor}/observations"
            ).value
            self.create_subscription(ConeObservations, topic, self.on_observations, qos)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(ConeMap, "/slam/map", map_qos)
        self.pose_pub = self.create_publisher(Odometry, "/slam/pose", 1)
        self.status_pub = self.create_publisher(SlamStatus, "/slam/status", 1)
        self.cones_pub = self.create_publisher(Cones, "/slam/planner_cones", 1)
        self.create_service(Trigger, "~/save_map", self.on_save)
        self.create_service(Trigger, "~/reset", self.on_reset)
        self.worker = threading.Thread(
            target=self.run_worker, name="cone_slam_worker", daemon=True
        )
        self.worker.start()
        self.create_timer(0.1, self.publish)

    def on_odom(self, msg: Odometry) -> None:
        """Validate the smooth odom/body contract and latch discontinuities."""
        try:
            if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
                raise ValueError("odometry_frames_require_odom_base_link")
            rot = quaternion_matrix(msg.pose.pose.orientation)
            p = msg.pose.pose.position
            cov = np.asarray(msg.pose.covariance).reshape(6, 6)[
                np.ix_([0, 1, 5], [0, 1, 5])
            ]
            # Zero covariance in common wheel odometry is unspecified, not perfect.
            if not np.any(cov):
                cov = np.diag([0.05**2, 0.05**2, 0.03**2])
            sample = OdomSample(
                seconds(msg.header.stamp),
                np.array([p.x, p.y, math.atan2(rot[1, 0], rot[0, 0])]),
                cov,
            )
            with self.lock:
                if self.odom.add(sample):
                    self.last_odom = msg
                    self.last_odom_wall = monotonic()
        except (ValueError, np.linalg.LinAlgError) as error:
            with self.lock:
                self.fault = str(error)

    def on_observations(self, msg: ConeObservations) -> None:
        """Queue raw batches without waiting for TF, optimization or disk I/O."""
        if (
            len(msg.observations) > 80
            or not msg.session_id
            or msg.source_sensor not in ("camera", "lidar")
        ):
            self.dropped += 1
            return
        try:
            self.jobs.put_nowait(("observations", (monotonic(), msg)))
        except queue.Full:
            self.dropped += 1

    def on_save(self, request: Any, response: Any) -> Any:
        """Queue an explicit save to the configured output path."""
        del request
        if not self.output_path or self.mode == "localization":
            response.success, response.message = (
                False,
                "Set output_map_path in mapping/update mode",
            )
            return response
        try:
            self.jobs.put_nowait(("save", None))
            response.success, response.message = (
                True,
                "Save queued; check /slam/status reason for completion",
            )
        except queue.Full:
            response.success, response.message = False, "Worker queue full"
        return response

    def on_reset(self, request: Any, response: Any) -> Any:
        """Explicitly start a new mapping session or relocalize the saved reference."""
        del request
        try:
            self.jobs.put_nowait(("reset", None))
            response.success, response.message = True, "Reset queued"
        except queue.Full:
            response.success, response.message = False, "Worker queue full"
        return response

    def _sensor_transform(
        self, frame: str, stamp: Any
    ) -> tuple[np.ndarray, np.ndarray]:
        if not frame:
            raise ValueError("missing_sensor_frame")
        tf = self.buffer.lookup_transform("base_link", frame, Time.from_msg(stamp))
        rot = quaternion_matrix(tf.transform.rotation)
        t = tf.transform.translation
        return rot, np.array([t.x, t.y, t.z])

    def _batch(self, msg: ConeObservations) -> bool:
        groups: dict[float, list[Observation]] = {}
        images = []
        for item in msg.observations:
            if item.source_sensor != msg.source_sensor or not item.observation_id:
                continue
            if not item.source_position_valid:
                if item.has_box and item.has_camera_info and item.color:
                    images.append(item)
                continue
            stamp = seconds(item.source_header.stamp)
            with self.lock:
                odom = self.odom.at(stamp)
            if odom is None:
                return False
            try:
                rot, translation = self._sensor_transform(
                    item.source_header.frame_id, item.source_header.stamp
                )
            except TransformException:
                return False
            p = item.source_position
            point = rot @ np.array([p.x, p.y, p.z]) + translation
            if item.covariance_known:
                try:
                    cov = covariance(np.array(item.source_covariance).reshape(3, 3), 3)
                except ValueError:
                    continue
                cov = (rot @ cov @ rot.T)[:2, :2]
            elif item.unavailable_reason == "invalid_covariance":
                continue
            else:
                # Range-dependent fallback includes uncalibrated centroid bias.
                sigma = self.settings.unknown_noise_m + 0.01 * np.linalg.norm(point[:2])
                cov = np.eye(2) * sigma**2
            calibration = hashlib.sha256(
                json.dumps(
                    [
                        rot.tolist(),
                        translation.tolist(),
                        list(item.camera_info.k) if item.has_camera_info else [],
                        self.settings.calibration_id,
                    ]
                ).encode()
            ).hexdigest()
            identity = f"{msg.session_id}/{msg.source_sensor}/{item.observation_id}"
            groups.setdefault(stamp, []).append(
                Observation(
                    identity,
                    msg.source_sensor,
                    stamp,
                    point[:2],
                    cov,
                    item.color,
                    item.confidence,
                    calibration,
                    item.source_header.frame_id,
                    {
                        "capture_time": stamp,
                        "source_frame": item.source_header.frame_id,
                        "source_position": [p.x, p.y, p.z],
                        "source_covariance": list(item.source_covariance)
                        if item.covariance_known
                        else None,
                        "sensor_to_body_rotation": rot.tolist(),
                        "sensor_to_body_translation": translation.tolist(),
                        "camera_projection": list(item.camera_info.p)
                        if item.has_camera_info
                        else None,
                    },
                )
            )
        for stamp, observations in sorted(groups.items()):
            with self.lock:
                odom = self.odom.at(stamp)
            if odom is not None:
                self.mapper.process(observations, odom)
        for item in images:
            self._image_color(item, msg.session_id)
        return True

    def _image_color(self, item: Any, session: str) -> None:
        if not self.mapper.localized or not self.mapper.graph.keys:
            return
        stamp = seconds(item.image_header.stamp)
        with self.lock:
            odom = self.odom.at(stamp)
        if (
            odom is None
            or item.camera_info.header.frame_id != item.image_header.frame_id
        ):
            return
        try:
            rot, translation = self._sensor_transform(
                item.image_header.frame_id, item.image_header.stamp
            )
        except (TransformException, ValueError):
            return
        pose = compose(self.mapper.graph.correction(), odom.pose)
        candidates = []
        projection = np.array(item.camera_info.p).reshape(3, 4)
        box = item.box
        if (
            not np.isfinite(projection).all()
            or not np.isfinite(box).all()
            or box[2] <= box[0]
            or box[3] <= box[1]
        ):
            return
        for identity, point in self.mapper.graph.points.items():
            body = np.r_[transform(inverse(pose), point), 0.15]
            camera = rot.T @ (body - translation)
            pixel = projection @ np.r_[camera, 1.0]
            if camera[2] > 0.1 and pixel[2] > 0.1:
                u, v = pixel[:2] / pixel[2]
                if box[0] <= u <= box[2] and box[1] <= v <= box[3]:
                    candidates.append(identity)
        if len(candidates) == 1:
            self.mapper.label(
                candidates[0],
                item.color,
                f"{session}/{item.observation_id}",
                "camera",
                len(self.mapper.graph.keys) - 1,
            )

    def _snapshot(self) -> None:
        mapper = self.mapper
        data = {
            "map_id": mapper.map_id,
            "revision": mapper.revision,
            "localized": mapper.localized,
            "tracking_ready": mapper.tracking_ready(
                self.get_clock().now().nanoseconds * 1e-9
            ),
            "reason": mapper.reason or mapper.graph.last_error,
            "last_observation": mapper.last_observation,
            "correction": mapper.graph.correction(),
            "pose_cov": mapper.graph.pose_cov.copy(),
            "landmarks": [
                (
                    deepcopy(value),
                    mapper.graph.points[key].copy(),
                    mapper.graph.point_cov[key].copy(),
                )
                for key, value in mapper.landmarks.items()
            ],
            "optimization_ms": mapper.graph.optimization_ms,
            "keyframes": len(mapper.graph.keys),
            "factors": len(mapper.graph.factors),
            "closures": mapper.closures,
            "closure_pending": mapper.closure_pending,
        }
        with self.lock:
            self.snapshot = data
            self.last_worker_wall = monotonic()

    def run_worker(self) -> None:
        """Serialize state mutations; retry missing capture-time TF/odom briefly."""
        pending: list[tuple[float, ConeObservations]] = []
        self._snapshot()
        while not self.stopping.is_set():
            try:
                kind, value = self.jobs.get(timeout=0.02)
            except queue.Empty:
                kind, value = "idle", None
            try:
                if kind == "reset":
                    self.mapper = (
                        load(self.map_path, self.settings, self.mode)
                        if self.map_path
                        else Mapper(self.settings, self.mode)
                    )
                    pending.clear()
                    with self.lock:
                        self.odom = OdomBuffer()
                        self.last_odom = None
                        self.fault = ""
                        self.last_clock = None
                    while not self.jobs.empty():
                        self.jobs.get_nowait()
                elif kind == "save":
                    save(self.mapper, self.output_path, self.source_commit)
                    self.mapper.reason = "map_saved"
                elif kind == "observations":
                    if len(pending) < 32:
                        pending.append(value)
                    else:
                        self.dropped += 1
                if not self.fault:
                    remaining = []
                    for arrival, msg in pending:
                        if monotonic() - arrival > 0.5:
                            self.dropped += 1
                        elif not self._batch(msg):
                            remaining.append((arrival, msg))
                    pending = remaining
                    self.mapper.age(self.get_clock().now().nanoseconds * 1e-9)
                else:
                    pending.clear()
                self._snapshot()
            except Exception as error:
                # A failed batch/save never kills the worker or silently makes
                # old outputs appear healthy. Reset is an explicit service.
                with self.lock:
                    self.fault = f"worker_error:{type(error).__name__}:{error}"
                self.get_logger().error(self.fault)

    def publish(self) -> None:
        """Publish map/health even when no pose or planner solution is available."""
        now = self.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        with self.lock:
            if self.last_clock is not None and stamp < self.last_clock - 0.001:
                self.fault = "clock_reset_requires_reset_service"
            self.last_clock = stamp
            data = self.snapshot
            odom = self.last_odom
            fault = self.fault
            odom_wall = self.last_odom_wall
            worker_wall = self.last_worker_wall
        if not data:
            return
        age = stamp - data["last_observation"]
        odom_age = stamp - seconds(odom.header.stamp) if odom else float("inf")
        available = bool(
            data["localized"]
            and data["tracking_ready"]
            and not fault
            and not data["closure_pending"]
            and 0 <= age <= self.settings.pose_timeout_s
            and 0 <= odom_age <= 0.2
            and monotonic() - odom_wall < 0.5
            and monotonic() - worker_wall < 1.0
            and data["reason"]
            not in ("capacity", "keyframe_capacity", "observation_capacity")
        )
        status = SlamStatus()
        status.header.stamp, status.header.frame_id = now.to_msg(), "map"
        status.mode, status.map_id, status.revision = (
            self.mode,
            data["map_id"],
            data["revision"],
        )
        status.map_available = any(
            row[0].state in ("confirmed", "inactive") for row in data["landmarks"]
        )
        status.pose_available = available
        status.quality = (
            "tracking"
            if available and not data["reason"]
            else "degraded"
            if available
            else "unavailable"
        )
        status.reason = (
            fault
            or data["reason"]
            or ("stale_pose_or_observations" if not available else "")
        )
        status.observation_age_s = float(age)
        status.optimization_ms = float(data["optimization_ms"])
        status.keyframes, status.factors = data["keyframes"], data["factors"]
        status.dropped_batches, status.accepted_closures = (
            self.dropped,
            data["closures"],
        )
        self.status_pub.publish(status)
        map_msg = ConeMap()
        map_msg.header = status.header
        map_msg.map_id, map_msg.revision = status.map_id, status.revision
        for landmark, point, cov in data["landmarks"]:
            row = ConeLandmark()
            row.id, row.state = landmark.id, landmark.state
            row.position.x, row.position.y = map(float, point)
            row.covariance = cov.ravel().tolist()
            row.existence_score, row.color = landmark.existence, landmark.color
            row.color_evidence = landmark.colors.astype(float).tolist()
            row.independent_hits = landmark.hits
            map_msg.landmarks.append(row)
        self.map_pub.publish(map_msg)
        local = Cones()
        local.header.stamp, local.header.frame_id = now.to_msg(), "odom"
        if available and odom is not None:
            p = odom.pose.pose.position
            rot = quaternion_matrix(odom.pose.pose.orientation)
            raw_pose = np.array([p.x, p.y, math.atan2(rot[1, 0], rot[0, 0])])
            pose = compose(data["correction"], raw_pose)
            output = deepcopy(odom)
            output.header.frame_id = "map"
            output.pose.pose.position.x, output.pose.pose.position.y = map(
                float, pose[:2]
            )
            output.pose.pose.orientation.x = output.pose.pose.orientation.y = 0.0
            output.pose.pose.orientation.z, output.pose.pose.orientation.w = (
                math.sin(pose[2] / 2),
                math.cos(pose[2] / 2),
            )
            cov = np.zeros((6, 6))
            cov[2, 2] = cov[3, 3] = cov[4, 4] = 1e6
            cov[np.ix_([0, 1, 5], [0, 1, 5])] = data["pose_cov"] + np.diag(
                [0.01, 0.01, 0.01]
            ) * max(age, 0)
            output.pose.covariance = cov.ravel().tolist()
            self.pose_pub.publish(output)
            if self.publish_tf:
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = (
                    odom.header.stamp,
                    "map",
                    "odom",
                )
                correction = data["correction"]
                tf.transform.translation.x, tf.transform.translation.y = map(
                    float, correction[:2]
                )
                tf.transform.rotation.z, tf.transform.rotation.w = (
                    math.sin(correction[2] / 2),
                    math.cos(correction[2] / 2),
                )
                self.broadcaster.sendTransform(tf)
            for landmark, point, _ in data["landmarks"]:
                if (
                    landmark.state != "confirmed"
                    or np.linalg.norm(point - pose[:2]) > self.settings.planner_range_m
                ):
                    continue
                position = transform(inverse(data["correction"]), point)
                cone = Cone()
                cone.x, cone.y, cone.color = (
                    float(position[0]),
                    float(position[1]),
                    landmark.color,
                )
                local.cones.append(cone)
        self.cones_pub.publish(local)

    def close(self) -> None:
        """Stop the worker before tearing down ROS entities."""
        self.stopping.set()
        self.worker.join(timeout=2.0)


def main(args: list[str] | None = None) -> None:
    """Run the standalone mapping/localization node."""
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
