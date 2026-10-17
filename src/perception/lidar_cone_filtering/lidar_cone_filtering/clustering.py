# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""Group the non-ground points into clusters with DBSCAN."""

import numpy as np
import open3d as o3d


def cluster_points(
    objects: o3d.geometry.PointCloud,
    cluster_eps: float,
    cluster_min_pts: int,
    min_cluster: int,
    max_cluster: int,
) -> list[np.ndarray]:
    """Return the points of every cluster whose size is within the limits."""
    objects_xyz = np.asarray(
        objects.points, dtype=np.float32
    )  # convert o3d object back to numpy

    clusters: list[np.ndarray] = []  # holds the points of each accepted cluster
    if len(objects.points) > 0:
        labels = np.array(
            objects.cluster_dbscan(
                eps=cluster_eps, min_points=cluster_min_pts, print_progress=False
            ),
            dtype=np.int32,
        )

        if labels.size > 0 and labels.max() >= 0:
            for k in range(labels.max() + 1):
                cluster_pts = objects_xyz[labels == k]
                n = cluster_pts.shape[0]

                if not (min_cluster <= n <= max_cluster):
                    continue

                clusters.append(cluster_pts)
    return clusters
