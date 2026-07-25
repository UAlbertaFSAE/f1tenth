"""Compute left/right track boundaries from a centerline via tangent/normal offsets."""
import numpy as np
from map_generator.geometry import left_normal, normalize


def compute_edges(centerline, width, closed=False):
    """centerline: (N,2) array. Returns (left, right) arrays, each (N,2).

    Left = +width/2 along the left-hand normal, right = -width/2.
    """
    n = len(centerline)
    if n < 2:
        return np.zeros((0, 2)), np.zeros((0, 2))

    tangents = np.zeros((n, 2))
    for i in range(n):
        if closed:
            prev_i = (i - 1) % n
            next_i = (i + 1) % n
        else:
            prev_i = max(i - 1, 0)
            next_i = min(i + 1, n - 1)
        tangents[i] = normalize(centerline[next_i] - centerline[prev_i])

    half = width / 2.0
    left = np.zeros((n, 2))
    right = np.zeros((n, 2))
    for i in range(n):
        normal = left_normal(tangents[i])
        left[i] = centerline[i] + half * normal
        right[i] = centerline[i] - half * normal

    return left, right
