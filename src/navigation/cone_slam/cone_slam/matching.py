"""Conservative one-to-one matching and bounded constellation relocalization."""

from itertools import combinations
from time import monotonic

import numpy as np
from scipy.spatial import cKDTree

from .geometry import rotation, transform, wrap


def associate(
    points: np.ndarray,
    targets: np.ndarray,
    covariances: list[np.ndarray],
    gate: float = 9.21,
    ambiguity: float = 2.0,
    max_distance: float = 0.8,
) -> tuple[dict[int, int], set[int]]:
    """Require mutual unambiguous Mahalanobis choices, withholding contested matches."""
    if not len(points) or not len(targets):
        return {}, set()
    cost = np.full((len(points), len(targets)), np.inf)
    for i, point in enumerate(points):
        delta = targets - point
        for j in np.flatnonzero(np.linalg.norm(delta, axis=1) <= max_distance):
            cost[i, j] = delta[j] @ np.linalg.solve(
                covariances[i * len(targets) + j], delta[j]
            )
    cost[cost > gate] = np.inf
    matches: dict[int, int] = {}
    withheld: set[int] = set()
    for i, row in enumerate(cost):
        possible = np.flatnonzero(np.isfinite(row))
        if not len(possible):
            continue
        order = possible[np.argsort(row[possible])]
        j = int(order[0])
        competitors = np.flatnonzero(
            np.isfinite(cost[:, j]) & (np.argmin(cost, axis=1) == j)
        )
        # Ambiguous nearby observations must not become new duplicate landmarks.
        if (len(order) > 1 and row[order[1]] - row[j] < ambiguity) or len(
            competitors
        ) != 1:
            withheld.add(i)
            continue
        matches[i] = j
    return matches, withheld


def constellation(
    local: np.ndarray,
    world: np.ndarray,
    local_colors: list[str],
    world_colors: list[str],
    tolerance: float = 0.25,
    budget_s: float = 0.08,
) -> tuple[np.ndarray | None, dict[int, int], str]:
    """Find a unique rigid constellation; return ambiguity on competing placements."""
    start = monotonic()
    if len(local) < 4 or len(world) < 4:
        return None, {}, "insufficient_constellation"
    if np.linalg.svd(local - local.mean(axis=0), compute_uv=False)[1] < 0.4:
        return None, {}, "degenerate_constellation"
    # Disjoint baselines prevent one distant outlier from being present in every
    # hypothesis. With >=75% required support, at least one such pair is inlier-only.
    # Exhaust the compatible map placements for ALL selected pairs before claiming
    # uniqueness; a timed-out search still returns unavailable.
    remaining = set(range(len(local)))
    anchors = []
    while len(remaining) >= 2:
        a, b = max(
            combinations(sorted(remaining), 2),
            key=lambda pair: np.linalg.norm(local[pair[1]] - local[pair[0]]),
        )
        anchors.append((a, b))
        remaining.difference_update((a, b))
    tree = cKDTree(world)
    solutions: list[tuple[int, float, np.ndarray, dict[int, int]]] = []
    for a, b in anchors:
        baseline = local[b] - local[a]
        length = np.linalg.norm(baseline)
        for u in range(len(world)):
            if monotonic() - start > budget_s:
                return None, {}, "constellation_budget"
            distances = np.linalg.norm(world - world[u], axis=1)
            for v in np.flatnonzero(np.abs(distances - length) < tolerance * 2):
                if u == v:
                    continue
                if any(
                    local_colors[i]
                    and world_colors[j]
                    and local_colors[i] != world_colors[j]
                    for i, j in ((a, u), (b, v))
                ):
                    continue
                direction = world[v] - world[u]
                yaw = wrap(
                    np.arctan2(direction[1], direction[0])
                    - np.arctan2(baseline[1], baseline[0])
                )
                pose = np.r_[world[u] - rotation(yaw) @ local[a], yaw]
                distance, nearest = tree.query(transform(pose, local), k=2)
                matches = {
                    i: int(nearest[i, 0])
                    for i in range(len(local))
                    if distance[i, 0] < tolerance
                    and distance[i, 1] - distance[i, 0] > tolerance * 0.5
                    and (
                        not local_colors[i]
                        or not world_colors[nearest[i, 0]]
                        or local_colors[i] == world_colors[nearest[i, 0]]
                    )
                }
                if len(matches) < max(4, int(np.ceil(0.75 * len(local)))) or len(
                    set(matches.values())
                ) != len(matches):
                    continue
                x = local[list(matches)]
                if np.linalg.svd(x - x.mean(axis=0), compute_uv=False)[1] < 0.4:
                    continue
                y = world[list(matches.values())]
                left, _, right = np.linalg.svd(
                    (x - x.mean(axis=0)).T @ (y - y.mean(axis=0))
                )
                rot = right.T @ left.T
                if np.linalg.det(rot) < 0:
                    continue
                pose = np.r_[
                    y.mean(axis=0) - rot @ x.mean(axis=0),
                    np.arctan2(rot[1, 0], rot[0, 0]),
                ]
                error = float(
                    np.sqrt(np.mean(np.sum((transform(pose, x) - y) ** 2, axis=1)))
                )
                if error <= tolerance * 0.6:
                    solutions.append((len(matches), error, pose, matches))
    if not solutions:
        return None, {}, "no_constellation"
    solutions.sort(key=lambda item: (-item[0], item[1]))
    best = solutions[0]
    for count, error, pose, _ in solutions[1:]:
        distinct = (
            np.linalg.norm(pose[:2] - best[2][:2]) > tolerance * 2
            or abs(wrap(pose[2] - best[2][2])) > 0.2
        )
        if distinct and count >= best[0] - 1 and error <= best[1] + tolerance * 0.3:
            return None, {}, "ambiguous_constellation"
    return best[2], best[3], ""
