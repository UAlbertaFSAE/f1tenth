"""Compute left/right track boundaries from a centerline via tangent/normal offsets."""

import numpy as np

from map_generator.geometry import left_normal, normalize

# Safety factor below the exact non-self-intersection threshold (offset ==
# local radius of curvature). Keeps the clamped inner cone a hair off the
# curve's own center of curvature instead of exactly on it.
_CURVATURE_SAFETY = 0.95

# How far apart (in meters, along the centerline) the 3 points used to
# estimate local curvature should be. Using immediate neighbors on a densely
# resampled spline (points ~0.1m apart) makes the curvature estimate wildly
# noise-sensitive -- three nearly-collinear points spaced 0.1m apart give an
# unstable circumradius, so the clamped offset swung point-to-point instead
# of shrinking smoothly, which is what read as "spikes" at corners.
_CURVATURE_BASELINE_M = 0.5


def _local_turn_radius(
    p_prev: np.ndarray, p_curr: np.ndarray, p_next: np.ndarray
) -> tuple[float, float]:
    """Local radius of curvature at p_curr, plus which side is concave.

    Circumradius of the 3-point triangle through p_prev, p_curr and p_next.

    Returns (radius, turn_sign): turn_sign > 0 means the path curves left
    here (left side is the inside/concave side of the turn), < 0 means it
    curves right (right side is concave). radius is np.inf for a
    straight/degenerate triple (no clamping needed).
    """
    a = np.linalg.norm(p_curr - p_next)
    b = np.linalg.norm(p_prev - p_next)
    c = np.linalg.norm(p_prev - p_curr)
    if a < 1e-9 or b < 1e-9 or c < 1e-9:
        return np.inf, 0.0

    v1 = p_curr - p_prev
    v2 = p_next - p_curr
    cross_z = v1[0] * v2[1] - v1[1] * v2[0]
    twice_area = abs(cross_z)
    if twice_area < 1e-9:
        return np.inf, 0.0

    radius = (a * b * c) / (2.0 * twice_area)
    turn_sign = 1.0 if cross_z > 0 else -1.0
    return radius, turn_sign


def _smooth(values: np.ndarray, window: int, closed: bool) -> np.ndarray:
    """Simple moving-average smoothing of the offset profile.

    Transitions gradually instead of jumping between adjacent samples.
    """
    if window < 2 or len(values) < window:
        return values
    kernel = np.ones(window) / window
    pad = window // 2
    mode: str = "wrap" if closed else "edge"
    padded = np.pad(values, pad, mode=mode)  # type: ignore[call-overload]
    smoothed = np.convolve(padded, kernel, mode="same")
    window_slice: np.ndarray = smoothed[pad : pad + len(values)]
    return window_slice


def compute_edges(
    centerline: np.ndarray, width: float, closed: bool = False
) -> tuple[np.ndarray, np.ndarray]:
    """centerline: (N,2) array. Returns (left, right) arrays, each (N,2).

    Left = +width/2 along the left-hand normal, right = -width/2, EXCEPT
    where the centerline turns tighter than width/2: naively offsetting a
    curve by a constant distance self-intersects on the concave (inside)
    side once the turn radius drops below that offset -- the inner boundary
    point ends up past the curve's own center of curvature, which is what
    turns a tight corner into a crossed triangle instead of a clean curve.
    The inner-side offset is clamped to the local turn radius (estimated
    over a stable arc-length baseline and smoothed), so it shrinks toward
    the centerline gradually as a corner tightens, instead of folding over
    or jumping around sample to sample.
    """
    n = len(centerline)
    if n < 2:
        return np.zeros((0, 2)), np.zeros((0, 2))

    diffs = np.linalg.norm(np.diff(centerline, axis=0), axis=1)
    avg_spacing = float(np.median(diffs)) if len(diffs) else 1.0
    avg_spacing = max(avg_spacing, 1e-6)
    look_steps = max(1, round(_CURVATURE_BASELINE_M / avg_spacing))
    smooth_window = 2 * look_steps + 1

    tangents = np.zeros((n, 2))
    radii = np.full(n, np.inf)
    turn_signs = np.zeros(n)
    for i in range(n):
        if closed:
            prev_i = (i - 1) % n
            next_i = (i + 1) % n
            far_prev_i = (i - look_steps) % n
            far_next_i = (i + look_steps) % n
        else:
            prev_i = max(i - 1, 0)
            next_i = min(i + 1, n - 1)
            far_prev_i = max(i - look_steps, 0)
            far_next_i = min(i + look_steps, n - 1)
        tangents[i] = normalize(centerline[next_i] - centerline[prev_i])
        radii[i], turn_signs[i] = _local_turn_radius(
            centerline[far_prev_i], centerline[i], centerline[far_next_i]
        )

    half = width / 2.0
    left_half = np.full(n, half)
    right_half = np.full(n, half)
    for i in range(n):
        clamped = min(half, _CURVATURE_SAFETY * radii[i])
        if turn_signs[i] > 0:  # curving left -> left side is concave
            left_half[i] = clamped
        elif turn_signs[i] < 0:  # curving right -> right side is concave
            right_half[i] = clamped

    left_half = _smooth(left_half, smooth_window, closed)
    right_half = _smooth(right_half, smooth_window, closed)
    # Re-clamp after smoothing -- averaging a clamped point with its
    # unclamped neighbors can nudge it back above the safe radius.
    left_half = np.minimum(left_half, half)
    right_half = np.minimum(right_half, half)

    left = np.zeros((n, 2))
    right = np.zeros((n, 2))
    for i in range(n):
        normal = left_normal(tangents[i])
        left[i] = centerline[i] + left_half[i] * normal
        right[i] = centerline[i] - right_half[i] * normal

    return left, right
