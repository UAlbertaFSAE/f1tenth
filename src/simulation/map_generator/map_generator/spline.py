"""Centripetal Catmull-Rom spline through control points, resampled to a
fixed arc-length resolution so the centerline has uniform point spacing.
"""
import numpy as np


def _tj(ti, pi, pj, alpha):
    d = np.linalg.norm(pj - pi)
    return ti + (d ** alpha if d > 1e-9 else 1e-6)


def _segment(p0, p1, p2, p3, n, alpha=0.5):
    """Catmull-Rom curve between p1 and p2, using p0/p3 as neighbors."""
    t0 = 0.0
    t1 = _tj(t0, p0, p1, alpha)
    t2 = _tj(t1, p1, p2, alpha)
    t3 = _tj(t2, p2, p3, alpha)

    ts = np.linspace(t1, t2, n, endpoint=False)
    pts = np.zeros((n, 2))
    for i, t in enumerate(ts):
        a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3

        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3

        c = (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2
        pts[i] = c
    return pts


def _padded_points(points, closed):
    pts = [np.array(p, dtype=float) for p in points]
    n = len(pts)
    if closed:
        return [pts[-1]] + pts + [pts[0], pts[1 % n]]
    # open track: mirror the endpoints so the curve doesn't need real neighbors
    first_phantom = pts[0] - (pts[1] - pts[0])
    last_phantom = pts[-1] - (pts[-2] - pts[-1])
    return [first_phantom] + pts + [last_phantom]


def generate_centerline(points, resolution=0.05, closed=False, samples_per_segment=20):
    """Return a dense, arc-length-uniform centerline sampled at `resolution` meters.

    points: list of (x, y) control points in meters.
    """
    points = [tuple(p) for p in points]
    n = len(points)
    if n < 2:
        return np.array(points, dtype=float).reshape(-1, 2)

    if n == 2:
        dense = np.array(points, dtype=float)
    else:
        padded = _padded_points(points, closed)
        segments = []
        span = n if closed else n - 1
        for i in range(span):
            p0, p1, p2, p3 = padded[i], padded[i + 1], padded[i + 2], padded[i + 3]
            segments.append(_segment(p0, p1, p2, p3, samples_per_segment))
        if closed:
            dense = np.vstack(segments)
        else:
            dense = np.vstack(segments + [padded[-2].reshape(1, 2)])

    return _resample_uniform(dense, resolution, closed)


def _resample_uniform(dense, resolution, closed):
    if closed and not np.allclose(dense[0], dense[-1]):
        dense = np.vstack([dense, dense[0]])

    diffs = np.diff(dense, axis=0)
    seg_len = np.linalg.norm(diffs, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cum[-1]
    if total < 1e-9:
        return dense

    n_out = max(2, int(total / resolution) + 1)
    targets = np.linspace(0.0, total, n_out, endpoint=not closed)

    out = np.zeros((len(targets), 2))
    j = 0
    for i, s in enumerate(targets):
        while j < len(cum) - 2 and cum[j + 1] < s:
            j += 1
        seg_s = cum[j]
        seg_e = cum[j + 1]
        frac = 0.0 if seg_e - seg_s < 1e-9 else (s - seg_s) / (seg_e - seg_s)
        out[i] = dense[j] + frac * (dense[j + 1] - dense[j])
    return out
