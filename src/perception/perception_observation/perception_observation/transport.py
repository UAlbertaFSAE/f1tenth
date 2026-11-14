"""Shared observation construction, exact-time transforms and bounded stream state."""

from collections import OrderedDict
from collections.abc import Sequence
from copy import deepcopy
from uuid import uuid4

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from rc_interfaces.msg import Cone, ConeObservation, ConeObservations, Cones
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

OBSERVATION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)


def stamp_ns(stamp: TimeMsg) -> int:
    """Return an integer ROS timestamp in nanoseconds."""
    return int(stamp.sec * 1_000_000_000 + stamp.nanosec)


def point(values: Sequence[float] | np.ndarray) -> Point:
    """Build a ROS point from three scalars."""
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def observation(
    header: Header,
    position: Sequence[float] | np.ndarray | None = None,
    *,
    covariance: Sequence[float] | np.ndarray | None = None,
    confidence: float = float("nan"),
    color: str = "",
    box: Sequence[float] | None = None,
    camera_info: CameraInfo | None = None,
    image_header: Header | None = None,
) -> ConeObservation:
    """Retain raw evidence and explicitly mark unknown or invalid quantities."""
    result = ConeObservation()
    result.inferred_color_confidence = float("nan")
    result.source_header = deepcopy(header)
    result.source_position = point(position if position is not None else [np.nan] * 3)
    result.source_position_valid = bool(
        position is not None and np.isfinite(position).all()
    )
    result.source_covariance = [float("nan")] * 9
    result.covariance = [float("nan")] * 9
    result.position = point([np.nan] * 3)
    result.confidence = (
        float(confidence)
        if np.isfinite(confidence) and 0 <= confidence <= 1
        else float("nan")
    )
    result.color = (
        color if color in ("blue", "yellow", "orange", "large_orange") else ""
    )
    if covariance is not None:
        cov = np.asarray(covariance, dtype=float)
        if cov.size == 9:
            result.source_covariance = cov.reshape(9).tolist()
            cov = cov.reshape(3, 3)
            result.covariance_known = bool(
                np.isfinite(cov).all()
                and np.allclose(cov, cov.T, atol=1e-10, rtol=0)
                and np.linalg.eigvalsh(cov).min() >= -1e-10
            )
        if not result.covariance_known:
            result.unavailable_reason = "invalid_covariance"
    if not result.source_position_valid:
        result.unavailable_reason = "invalid_position"
    if (
        box is not None
        and len(box) == 4
        and np.isfinite(box).all()
        and box[2] > box[0]
        and box[3] > box[1]
    ):
        result.image_header = deepcopy(
            image_header if image_header is not None else header
        )
        result.has_box = True
        result.box = [float(v) for v in box]
    if camera_info is not None:
        result.has_camera_info = True
        result.camera_info = deepcopy(camera_info)
    return result


def transform_observation(
    item: ConeObservation, target_frame: str, buffer: Buffer
) -> ConeObservation:
    """Transform point and covariance with TF at capture time, never time zero."""
    item.header = deepcopy(item.source_header)
    item.header.frame_id = target_frame
    item.position_valid = False
    item.position = point([np.nan] * 3)
    item.covariance = [float("nan")] * 9
    if (
        stamp_ns(item.source_header.stamp) <= 0
        or not item.source_header.frame_id
        or not target_frame
    ):
        item.unavailable_reason = "invalid_header"
    if item.unavailable_reason:
        return item
    rotation = np.eye(3)
    translation = np.zeros(3)
    if target_frame != item.source_header.frame_id:
        try:
            tf = buffer.lookup_transform(
                target_frame,
                item.source_header.frame_id,
                Time.from_msg(item.source_header.stamp),
                timeout=Duration(),
            ).transform
        except TransformException:
            item.unavailable_reason = "missing_transform"
            return item
        q = np.array([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
        translation = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        norm = np.linalg.norm(q)
        if (
            not np.isfinite(q).all()
            or norm < 1e-12
            or not np.isfinite(translation).all()
        ):
            item.unavailable_reason = "invalid_transform"
            return item
        x, y, z, w = q / norm
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
    raw = item.source_position
    transformed: np.ndarray = rotation @ [raw.x, raw.y, raw.z] + translation
    if not np.isfinite(transformed).all():
        item.unavailable_reason = "invalid_position"
        return item
    item.position = point(transformed)
    item.position_valid = True
    if item.covariance_known:
        cov = np.asarray(item.source_covariance).reshape(3, 3)
        item.covariance = (rotation @ cov @ rotation.T).reshape(9).tolist()
    return item


def infer_heading_color(item: ConeObservation, buffer: Buffer) -> None:
    """Infer left yellow/right blue in capture-time base_link, retaining raw color."""
    item.inferred_color = ""
    item.color_inference_method = ""
    item.inferred_color_confidence = float("nan")
    if not item.source_position_valid:
        return
    # Use a copy so a failed heading lookup cannot invalidate odom coordinates.
    car_relative = transform_observation(deepcopy(item), "base_link", buffer)
    if not car_relative.position_valid or abs(car_relative.position.y) <= 1e-6:
        return
    item.inferred_color = "yellow" if car_relative.position.y > 0 else "blue"
    item.color_inference_method = "car_heading_left_right"


class StreamState:
    """Deduplicate capture frames with bounded memory; distinguish clock epochs."""

    def __init__(self, capacity: int = 256) -> None:
        """Keep at most capacity frame keys, accepting bounded reordering."""
        self.capacity = capacity
        self.session_id = uuid4().hex
        self.seen: OrderedDict[tuple[str, int], None] = OrderedDict()
        self.floor = -1
        self.last_now: int | None = None

    def accept(self, header: Header, now_ns: int) -> bool:
        """Reject duplicates/expired frames; reset only when the ROS clock rewinds."""
        if self.last_now is not None and now_ns < self.last_now:
            self.session_id = uuid4().hex
            self.seen.clear()
            self.floor = -1
        self.last_now = now_ns
        stamp = stamp_ns(header.stamp)
        key = (header.frame_id, stamp)
        if key in self.seen or stamp <= self.floor:
            return False
        self.seen[key] = None
        if len(self.seen) > self.capacity:
            old, _ = self.seen.popitem(last=False)
            self.floor = max(self.floor, old[1])
        return True


def legacy_cones(batch: ConeObservations, target_frame: str) -> Cones:
    """Adapt valid, known-color camera observations for existing planner consumers."""
    result = Cones()
    valid = [item for item in batch.observations if item.position_valid and item.color]
    result.header = deepcopy(valid[0].header if valid else batch.header)
    result.header.frame_id = target_frame
    result.cones = [
        Cone(x=item.position.x, y=item.position.y, color=item.color) for item in valid
    ]
    return result


class ObservationPublisher:
    """Embed the same observation transport in either existing detector node."""

    def __init__(
        self, node: Node, sensor: str, topic: str, target_frame: str = "odom"
    ) -> None:
        """Create reliable bounded output and a listener for capture-time TF."""
        self.node = node
        self.sensor = sensor
        self.target_frame = target_frame
        self.state = StreamState()
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.publisher = node.create_publisher(ConeObservations, topic, OBSERVATION_QOS)

    def publish(
        self, header: Header, items: list[ConeObservation], *, infer_color: bool = False
    ) -> ConeObservations | None:
        """Publish one batch, including unavailable measurements; suppress duplicates."""
        previous_session = self.state.session_id
        if not self.state.accept(header, self.node.get_clock().now().nanoseconds):
            return None
        if previous_session != self.state.session_id:
            self.buffer.clear()
        batch = ConeObservations()
        batch.header = deepcopy(header)
        batch.source_sensor = self.sensor
        batch.session_id = self.state.session_id
        for index, item in enumerate(items):
            item.source_sensor = self.sensor
            item.observation_id = f"{batch.session_id}/{self.sensor}/{header.frame_id}/{stamp_ns(header.stamp)}/{index}"
            if infer_color:
                infer_heading_color(item, self.buffer)
            transform_observation(
                item, self.target_frame or header.frame_id, self.buffer
            )
        batch.observations = items
        self.publisher.publish(batch)
        return batch
