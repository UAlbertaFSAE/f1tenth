"""Planar geometry and validated capture-time odometry, independent of ROS."""

from bisect import bisect_left
from dataclasses import dataclass
from typing import cast

import numpy as np


def wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def rotation(yaw: float) -> np.ndarray:
    """Return the planar rotation matrix."""
    c, s = np.cos(yaw), np.sin(yaw)
    return cast(np.ndarray, np.array([[c, -s], [s, c]]))


def transform(pose: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Transform local points into the pose's parent frame."""
    return cast(np.ndarray, points @ rotation(float(pose[2])).T + pose[:2])


def inverse(pose: np.ndarray) -> np.ndarray:
    """Invert an SE(2) pose."""
    return cast(np.ndarray, np.r_[-rotation(float(pose[2])).T @ pose[:2], -pose[2]])


def compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Compose two SE(2) poses."""
    return cast(np.ndarray, np.r_[transform(a, b[:2]), wrap(float(a[2] + b[2]))])


def between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Express b relative to a."""
    return compose(inverse(a), b)


def covariance(value: np.ndarray, size: int, floor: float = 1e-6) -> np.ndarray:
    """Validate covariance and impose a strictly positive noise floor."""
    value = np.asarray(value, dtype=float)
    if value.shape != (size, size) or not np.isfinite(value).all():
        raise ValueError("nonfinite_or_wrong_covariance")
    if (
        not np.allclose(value, value.T, atol=1e-8)
        or np.linalg.eigvalsh(value).min() < -1e-8
    ):
        raise ValueError("invalid_covariance")
    return cast(np.ndarray, value + np.eye(size) * floor)


@dataclass
class OdomSample:
    """Validated planar odometry at one capture timestamp."""

    stamp: float
    pose: np.ndarray
    cov: np.ndarray


class OdomBuffer:
    """Bounded interpolation buffer; never extrapolate or silently reset a graph."""

    def __init__(self, horizon: float = 10.0, max_gap: float = 0.2) -> None:
        """Set bounded history and the maximum interpolation gap."""
        self.horizon = horizon
        self.max_gap = max_gap
        self.samples: list[OdomSample] = []

    def add(self, sample: OdomSample) -> bool:
        """Reject out-of-order, nonfinite and physically implausible odometry."""
        if (
            not np.isfinite(sample.stamp)
            or sample.stamp < 0
            or not np.isfinite(sample.pose).all()
        ):
            raise ValueError("invalid_odometry")
        sample.cov = covariance(sample.cov, 3)
        if self.samples:
            last = self.samples[-1]
            dt = sample.stamp - last.stamp
            if dt <= 0:
                return False
            delta = between(last.pose, sample.pose)
            if (
                np.linalg.norm(delta[:2]) > 15 * dt + 0.1
                or abs(delta[2]) > 5 * dt + 0.1
            ):
                raise ValueError("odometry_discontinuity")
        self.samples.append(sample)
        cutoff = sample.stamp - self.horizon
        while len(self.samples) > 2 and (
            self.samples[1].stamp < cutoff or len(self.samples) > 4000
        ):
            self.samples.pop(0)
        return True

    def at(self, stamp: float) -> OdomSample | None:
        """Interpolate only inside a sufficiently close pair of measured poses."""
        times = [sample.stamp for sample in self.samples]
        i = bisect_left(times, stamp)
        if i < len(times) and abs(times[i] - stamp) < 1e-8:
            return self.samples[i]
        if i == 0 or i == len(times):
            return None
        a, b = self.samples[i - 1], self.samples[i]
        if b.stamp - a.stamp > self.max_gap:
            return None
        fraction = (stamp - a.stamp) / (b.stamp - a.stamp)
        pose = (
            a.pose
            + fraction * np.r_[b.pose[:2] - a.pose[:2], wrap(b.pose[2] - a.pose[2])]
        )
        return OdomSample(stamp, pose, (1 - fraction) * a.cov + fraction * b.cov)
