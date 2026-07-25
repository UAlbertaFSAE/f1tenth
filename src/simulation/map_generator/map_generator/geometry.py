"""Vector helpers. All units meters, points are 2-element numpy arrays."""
import numpy as np


def to_vec(p):
    return np.array([p[0], p[1]], dtype=float)


def distance(p1, p2):
    return float(np.linalg.norm(to_vec(p2) - to_vec(p1)))


def normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-9:
        return np.array([0.0, 0.0])
    return v / n


def tangent(p_prev, p_next):
    return normalize(to_vec(p_next) - to_vec(p_prev))


def left_normal(t):
    """90 deg CCW rotation of a tangent -> left-hand side normal."""
    return np.array([-t[1], t[0]])


def polyline_length(points):
    if len(points) < 2:
        return 0.0
    pts = np.asarray(points, dtype=float)
    diffs = np.diff(pts, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))
