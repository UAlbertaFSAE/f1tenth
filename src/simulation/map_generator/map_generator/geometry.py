"""Vector helpers. All units meters, points are 2-element numpy arrays."""

from collections.abc import Sequence

import numpy as np


def to_vec(p: Sequence[float] | np.ndarray) -> np.ndarray:
    """Coerce a point-like to a 2-element float vector."""
    vec: np.ndarray = np.array([p[0], p[1]], dtype=float)
    return vec


def distance(
    p1: Sequence[float] | np.ndarray, p2: Sequence[float] | np.ndarray
) -> float:
    """Euclidean distance between two points."""
    return float(np.linalg.norm(to_vec(p2) - to_vec(p1)))


def normalize(v: np.ndarray) -> np.ndarray:
    """Unit vector in the direction of ``v``, or zero if ``v`` is degenerate."""
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.zeros(2)
    scaled: np.ndarray = v / n
    return scaled


def tangent(
    p_prev: Sequence[float] | np.ndarray, p_next: Sequence[float] | np.ndarray
) -> np.ndarray:
    """Unit tangent through a point, from its neighbours."""
    return normalize(to_vec(p_next) - to_vec(p_prev))


def left_normal(t: np.ndarray) -> np.ndarray:
    """90 deg CCW rotation of a tangent -> left-hand side normal."""
    normal: np.ndarray = np.array([-t[1], t[0]])
    return normal


def polyline_length(points: Sequence[Sequence[float]] | np.ndarray) -> float:
    """Total arc length along a polyline."""
    if len(points) < 2:
        return 0.0
    pts = np.asarray(points, dtype=float)
    diffs = np.diff(pts, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))
