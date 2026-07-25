"""Place blue/yellow cone pairs at regular arc-length intervals along the track."""
import numpy as np


def _arc_length_indices(centerline, spacing, closed):
    n = len(centerline)
    if n < 2:
        return []

    pts = centerline
    if closed and not np.allclose(pts[0], pts[-1]):
        pts = np.vstack([pts, pts[0]])

    diffs = np.diff(pts, axis=0)
    seg_len = np.linalg.norm(diffs, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cum[-1]
    if total < 1e-9:
        return [0]

    n_stations = max(1, int(total / spacing) + (0 if closed else 1))
    targets = np.linspace(0.0, total, n_stations, endpoint=not closed) if closed else \
        np.arange(0.0, total + 1e-9, spacing)

    indices = []
    j = 0
    for s in targets:
        while j < len(cum) - 2 and cum[j + 1] < s:
            j += 1
        indices.append(j)
    return indices


def generate_cone_pairs(centerline, left, right, spacing=1.0, closed=False):
    """Returns list of dicts: {id, blue: (x,y), yellow: (x,y)}."""
    if len(centerline) < 2:
        return []

    indices = _arc_length_indices(centerline, spacing, closed)
    pairs = []
    for pair_id, idx in enumerate(indices):
        idx = min(idx, len(left) - 1)
        pairs.append({
            "id": pair_id,
            "blue": (float(left[idx][0]), float(left[idx][1])),
            "yellow": (float(right[idx][0]), float(right[idx][1])),
        })
    return pairs
