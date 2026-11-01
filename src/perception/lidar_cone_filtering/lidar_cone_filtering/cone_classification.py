# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""Decide which clusters are shaped like cones."""

import numpy as np


def is_cone_cluster(
    cluster_pts: np.ndarray,
    small_height: float,
    large_height: float,
    tolerance: float,
    small_width: float,
    large_width: float,
    width_tolerance: float,
    min_height: float,
) -> bool:
    """Determines whether cluster resembles a cone."""
    z_min = float(np.min(cluster_pts[:, 2]))
    z_max = float(np.max(cluster_pts[:, 2]))
    height = z_max - z_min

    x_min = float(np.min(cluster_pts[:, 0]))
    x_max = float(np.max(cluster_pts[:, 0]))

    y_min = float(np.min(cluster_pts[:, 1]))
    y_max = float(np.max(cluster_pts[:, 1]))

    x_width = x_max - x_min
    y_width = y_max - y_min

    width = max(x_width, y_width)

    # Only upper limits on width, a far cone can show as a few points close together
    small_ok = (
        min_height <= height <= small_height + tolerance
        and width <= small_width + width_tolerance
    )

    large_ok = (
        abs(height - large_height) <= tolerance
        and abs(width - large_width) <= width_tolerance
    )

    return small_ok or large_ok


def cone_centroids(
    clusters: list[np.ndarray],
    small_height: float,
    large_height: float,
    tolerance: float,
    small_width: float,
    large_width: float,
    width_tolerance: float,
    min_height: float,
) -> list[np.ndarray]:
    """Return the centroid of every cluster that is shaped like a cone."""
    centroids: list[
        np.ndarray
    ] = []  # holds one [x, y, z] centroid per accepted cluster
    for cluster_pts in clusters:
        if not is_cone_cluster(
            cluster_pts,
            small_height,
            large_height,
            tolerance,
            small_width,
            large_width,
            width_tolerance,
            min_height,
        ):
            continue

        centroids.append(cluster_pts.mean(axis=0))
    return centroids


def remove_isolated_cones(
    centroids: list[np.ndarray],
    neighbor_radius: float,
    min_neighbors: int,
) -> list[np.ndarray]:
    """Return the centroids with enough other cones nearby, measured in the XY plane."""
    if not centroids:
        return centroids

    xy = np.array(centroids)[:, :2]
    distances = np.linalg.norm(xy[:, None] - xy[None], axis=2)
    neighbor_counts = np.sum(distances <= neighbor_radius, axis=1) - 1  # skip itself

    return [
        c for c, n in zip(centroids, neighbor_counts, strict=True) if n >= min_neighbors
    ]
