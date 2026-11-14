"""Persistent landmarks, asynchronous fusion and GraphSLAM orchestration."""

from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any
from uuid import uuid4

import numpy as np

from .geometry import (
    OdomSample,
    between,
    compose,
    covariance,
    inverse,
    rotation,
    transform,
)
from .graph import Factor, Graph, Keyframe
from .matching import associate, constellation

COLORS = ("blue", "yellow", "orange", "large_orange")


@dataclass
class Settings:
    """Explicit bounded defaults; runtime targets require Jetson validation."""

    max_keyframes: int = 600
    max_landmarks: int = 1000
    max_factors: int = 12000
    key_distance: float = 0.5
    key_angle: float = 0.2
    key_seconds: float = 1.0
    confirmation_hits: int = 3
    tentative_ttl: float = 3.0
    inactive_after: float = 15.0
    association_distance: float = 0.8
    closure_after: float = 12.0
    closure_translation: float = 3.0
    closure_angle: float = 0.6
    closure_confirmations: int = 3
    optimization_budget_s: float = 0.25
    optimization_interval_s: float = 1.0
    unknown_noise_m: float = 0.20
    calibration_noise_m: float = 0.08
    max_range_m: float = 20.0
    min_range_m: float = 0.2
    planner_range_m: float = 8.0
    pose_timeout_s: float = 1.0
    calibration_id: str = "uncalibrated-cone-base-xy-v1"

    def validate(self) -> None:
        """Reject invalid limits before allocating state or processing data."""
        for name, value in vars(self).items():
            if isinstance(value, (int, float)) and (
                not np.isfinite(value) or value <= 0
            ):
                raise ValueError(f"invalid setting: {name}")
        if (
            self.max_keyframes < 2
            or self.confirmation_hits < 2
            or self.closure_confirmations < 2
        ):
            raise ValueError("insufficient confirmation/capacity")
        if self.min_range_m >= self.max_range_m:
            raise ValueError("invalid range")


@dataclass
class Observation:
    """Body-relative raw evidence after exact-time sensor extrinsic conversion."""

    id: str
    sensor: str
    stamp: float
    point: np.ndarray
    cov: np.ndarray
    color: str = ""
    confidence: float = float("nan")
    calibration: str = ""
    frame: str = ""
    provenance: dict[str, Any] = field(default_factory=dict)


@dataclass
class Landmark:
    """Position, existence and measured-color evidence are separate quantities."""

    id: int
    state: str = "tentative"
    hits: int = 0
    first_seen: float = 0.0
    last_seen: float = 0.0
    last_key: int = -1
    colors: np.ndarray = field(default_factory=lambda: np.zeros(4))
    color_keys: dict[str, int] = field(default_factory=dict)
    misses: int = 0

    @property
    def color(self) -> str:
        """Return unknown until accumulated measured color is dominant."""
        total = self.colors.sum()
        return (
            COLORS[int(np.argmax(self.colors))]
            if total >= 2 and self.colors.max() / total >= 0.75
            else ""
        )

    @property
    def existence(self) -> float:
        """Return an evidence score, explicitly not a calibrated probability."""
        return float(self.hits / (self.hits + self.misses + 2))


class Mapper:
    """Single-worker mapper; ROS callbacks never mutate its graph concurrently."""

    def __init__(self, settings: Settings | None = None, mode: str = "mapping") -> None:
        """Initialize independent mapping or an unlocalized saved-map session."""
        self.settings = settings or Settings()
        self.settings.validate()
        if mode not in ("mapping", "localization", "update"):
            raise ValueError("invalid_mode")
        self.mode = mode
        self.graph = Graph()
        self.landmarks: dict[int, Landmark] = {}
        self.map_id = uuid4().hex
        self.next_id = 1
        self.revision = 0
        self.localized = mode == "mapping"
        self.reason = "waiting_for_observations"
        self.last_observation = -float("inf")
        self.last_optimization = -float("inf")
        self.last_solution = -float("inf")
        self.last_search = -float("inf")
        self.seen: OrderedDict[str, None] = OrderedDict()
        self.pending_correction: np.ndarray | None = None
        self.pending_odom: np.ndarray | None = None
        self.pending_count = 0
        self.closures = 0
        self.closure_pending = False
        self.last_batch = -float("inf")
        self.reference_ids: set[int] = set()

    def tracking_ready(self, stamp: float) -> bool:
        """Require recent evidence and a committed solution, independent of paths."""
        return bool(
            self.localized
            and not self.closure_pending
            and 0 <= stamp - self.last_observation <= self.settings.pose_timeout_s
            and 0
            <= stamp - self.last_solution
            <= max(2.0, 2.5 * self.settings.optimization_interval_s)
            and self.reason
            not in ("capacity", "keyframe_capacity", "observation_capacity")
        )

    def _remember(self, identity: str) -> bool:
        if identity in self.seen:
            return False
        self.seen[identity] = None
        while len(self.seen) > 20000:
            self.seen.popitem(last=False)
        return True

    def age(self, stamp: float) -> None:
        """Expire tentative clutter; unseen confirmed landmarks retain identity."""
        retired = []
        for landmark in self.landmarks.values():
            if landmark.id in self.reference_ids:
                continue
            elapsed = stamp - landmark.last_seen
            if landmark.state == "tentative" and elapsed > self.settings.tentative_ttl:
                retired.append(landmark.id)
            elif (
                landmark.state == "confirmed" and elapsed > self.settings.inactive_after
            ):
                landmark.state = "inactive"
        if retired:
            self.graph.factors = [
                f for f in self.graph.factors if f.landmark not in retired
            ]
            for identity in retired:
                del self.landmarks[identity]
                self.graph.points.pop(identity, None)
                self.graph.point_cov.pop(identity, None)
            # IDs are never recycled. Retired history is not retained unboundedly.
            self.revision += 1

    def negative_evidence(
        self,
        ids: list[int],
        *,
        visible: bool,
        unoccluded: bool,
        in_range: bool,
        healthy: bool,
    ) -> None:
        """Apply misses only from an explicitly qualified visibility assessment."""
        if (
            not all((visible, unoccluded, in_range, healthy))
            or self.mode == "localization"
        ):
            return
        for identity in set(ids):
            landmark = self.landmarks.get(identity)
            if landmark is not None and identity not in self.reference_ids:
                landmark.misses += 1
                if landmark.misses >= 5:
                    landmark.state = "suspect"

    def _global(
        self, observations: list[Observation], odom: OdomSample
    ) -> tuple[np.ndarray | None, dict[int, int]]:
        eligible = [
            k
            for k, value in self.landmarks.items()
            if value.state != "tentative"
            and (
                not self.localized
                or odom.stamp - value.last_seen > self.settings.closure_after
            )
        ]
        if len(eligible) < 4 or len(observations) < 4:
            return None, {}
        pose, matches, reason = constellation(
            np.array([o.point for o in observations]),
            np.array([self.graph.points[k] for k in eligible]),
            [o.color for o in observations],
            [self.landmarks[k].color for k in eligible],
        )
        if pose is None:
            self.pending_count = 0
            if not self.localized:
                self.reason = reason
            return None, {}
        correction = compose(pose, inverse(odom.pose))
        if self.localized:
            delta = between(compose(self.graph.correction(), odom.pose), pose)
            if (
                np.linalg.norm(delta[:2]) > self.settings.closure_translation
                or abs(delta[2]) > self.settings.closure_angle
            ):
                self.pending_count = 0
                return None, {}
        if (
            self.pending_correction is None
            or np.linalg.norm(between(self.pending_correction, correction)[:2]) > 0.3
            or abs(between(self.pending_correction, correction)[2]) > 0.15
        ):
            self.pending_count = 1
        elif (
            self.pending_odom is not None
            and np.linalg.norm(between(self.pending_odom, odom.pose)[:2]) >= 0.15
        ):
            self.pending_count += 1
        else:
            return None, {}
        self.pending_correction, self.pending_odom = correction, odom.pose.copy()
        if self.pending_count < self.settings.closure_confirmations:
            self.reason = (
                "validating_constellation" if not self.localized else self.reason
            )
            return None, {}
        self.pending_count = 0
        return pose, {i: eligible[j] for i, j in matches.items()}

    def process(self, observations: list[Observation], odom: OdomSample) -> None:
        """Associate one sensor capture batch without synchronizing camera and lidar."""
        settings = self.settings
        # Bounded reorder tolerance prevents replaying evicted duplicate IDs.
        if odom.stamp < self.last_batch - 1.0:
            self.reason = "late_batch"
            return
        self.last_batch = max(self.last_batch, odom.stamp)
        self.age(odom.stamp)
        valid = []
        for item in sorted(observations, key=lambda value: value.id):
            if (
                not item.id
                or not np.isfinite(item.point).all()
                or not settings.min_range_m
                <= np.linalg.norm(item.point)
                <= settings.max_range_m
            ):
                continue
            if np.isfinite(item.confidence) and (
                item.confidence < 0.3 or item.confidence > 1
            ):
                continue
            try:
                item.cov = covariance(item.cov, 2, settings.calibration_noise_m**2)
            except ValueError:
                continue
            if self._remember(item.id):
                valid.append(item)
        if not valid:
            return
        # A batch larger than this is not truncated into a misleading constellation.
        if len(valid) > 80:
            self.reason = "observation_capacity"
            return
        global_matches: dict[int, int] = {}
        global_pose = None
        if odom.stamp - self.last_search >= 0.5:
            self.last_search = odom.stamp
            global_pose, global_matches = self._global(valid, odom)
        if not self.localized:
            if global_pose is None:
                return
            self.localized = True
        if not self.graph.keys:
            pose = global_pose if global_pose is not None else np.zeros(3)
            self.graph.add_key(
                Keyframe(odom.stamp, odom.pose.copy(), pose.copy(), odom.cov.copy())
            )
        delta = between(self.graph.keys[-1].odom, odom.pose)
        needs_key = (
            np.linalg.norm(delta[:2]) >= settings.key_distance
            or abs(delta[2]) >= settings.key_angle
            or odom.stamp - self.graph.keys[-1].stamp >= settings.key_seconds
        )
        if needs_key:
            if len(self.graph.keys) >= settings.max_keyframes:
                self.reason = "keyframe_capacity"
                return
            self.graph.add_key(
                Keyframe(
                    odom.stamp,
                    odom.pose.copy(),
                    compose(self.graph.correction(), odom.pose),
                    odom.cov.copy(),
                )
            )
        # Delayed observations attach to the closest preceding retained keyframe.
        candidates = [
            i for i, key in enumerate(self.graph.keys) if key.stamp <= odom.stamp
        ]
        if not candidates:
            self.reason = "before_first_keyframe"
            return
        key = candidates[-1]
        offset = between(self.graph.keys[key].odom, odom.pose)
        predicted = compose(self.graph.keys[key].pose, offset)
        world = np.array([transform(predicted, o.point) for o in valid])
        local_ids = [
            k
            for k, value in self.landmarks.items()
            if value.state != "suspect"
            and (
                odom.stamp - value.last_seen <= settings.closure_after
                or k in self.reference_ids
            )
        ]
        targets = np.array([self.graph.points[k] for k in local_ids])
        covs = []
        for item in valid:
            rot = rotation(float(predicted[2]))
            pose_var = max(float(np.trace(self.graph.pose_cov[:2, :2])), 0.04)
            for local_id in local_ids:
                covs.append(
                    rot @ item.cov @ rot.T
                    + self.graph.point_cov[local_id]
                    + np.eye(2) * pose_var
                )
        matches, withheld = associate(
            world, targets, covs, max_distance=settings.association_distance
        )
        matched = {i: local_ids[j] for i, j in matches.items()}
        if global_matches:
            matched = global_matches
            self.closure_pending = bool(self.graph.keys and len(self.graph.keys) > 1)
        existing_ids = set(self.landmarks)
        accepted = 0
        capacity = False
        for i, item in enumerate(valid):
            identity = matched.get(i)
            if identity is None:
                if i in withheld or self.mode == "localization" or global_matches:
                    continue
                # Spatially nearby unmatched/inactive cones are ambiguous evidence,
                # not permission to duplicate a persistent physical landmark.
                if any(
                    np.linalg.norm(world[i] - p)
                    < (settings.association_distance if k in existing_ids else 0.15)
                    for k, p in self.graph.points.items()
                ):
                    continue
                if len(self.landmarks) >= settings.max_landmarks:
                    capacity = True
                    continue
                identity = self.next_id
                self.next_id += 1
                self.landmarks[identity] = Landmark(
                    identity, first_seen=odom.stamp, last_seen=odom.stamp
                )
                self.graph.points[identity] = world[i]
                self.graph.point_cov[identity] = item.cov + np.eye(2) * 0.25
            landmark = self.landmarks[identity]
            if self.mode == "localization" and landmark.state == "inactive":
                # Saved inactivity describes the old session. A validated current
                # match restores live usability without changing reference geometry.
                landmark.state = "confirmed"
            independent = key > landmark.last_key
            if independent and len(self.graph.factors) >= settings.max_factors:
                capacity = True
                continue
            if independent:
                # One positional factor per physical landmark/keyframe across
                # sensors: overlapping raw/fused/depth evidence cannot multiply it.
                rot = rotation(float(offset[2]))
                interpolation_noise = (
                    np.eye(2) * (0.02 * np.linalg.norm(offset[:2])) ** 2
                )
                self.graph.factors.append(
                    Factor(
                        key,
                        identity,
                        item.point.copy(),
                        offset.copy(),
                        rot @ item.cov @ rot.T + interpolation_noise,
                        item.id,
                        item.sensor,
                        item.calibration,
                        item.provenance,
                    )
                )
                landmark.last_key = key
                if self.mode != "localization":
                    landmark.hits += 1
                    if landmark.hits >= settings.confirmation_hits:
                        landmark.state = "confirmed"
                accepted += 1
            if (
                self.mode != "localization"
                and item.color in COLORS
                and key > landmark.color_keys.get(item.sensor, -1)
            ):
                landmark.colors[COLORS.index(item.color)] += 1
                landmark.color_keys[item.sensor] = key
            landmark.last_seen = max(landmark.last_seen, odom.stamp)
        if matched or accepted:
            self.last_observation = max(self.last_observation, odom.stamp)
            self.reason = "capacity" if capacity else ""
            self.revision += self.mode != "localization"
        if (
            odom.stamp - self.last_optimization >= settings.optimization_interval_s
            or global_matches
        ):
            self.last_optimization = odom.stamp
            if self.graph.optimize(settings.optimization_budget_s):
                self.last_solution = self.last_batch
                if self.closure_pending:
                    self.closures += 1
                    self.closure_pending = False
                self.revision += self.mode != "localization"
            else:
                self.reason = self.graph.last_error

    def label(
        self, identity: int, color: str, evidence: str, sensor: str, key: int
    ) -> None:
        """Apply uniquely projected image-only color without adding position/existence."""
        landmark = self.landmarks.get(identity)
        if (
            self.mode == "localization"
            or landmark is None
            or color not in COLORS
            or not self._remember(evidence)
        ):
            return
        if key > landmark.color_keys.get(sensor, -1):
            landmark.colors[COLORS.index(color)] += 1
            landmark.color_keys[sensor] = key
            self.revision += 1
