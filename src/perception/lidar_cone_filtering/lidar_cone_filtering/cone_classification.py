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

    small_ok = (
        abs(height - small_height) <= tolerance
        and abs(width - small_width) <= width_tolerance
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
        ):
            continue

        centroids.append(cluster_pts.mean(axis=0))
    return centroids
