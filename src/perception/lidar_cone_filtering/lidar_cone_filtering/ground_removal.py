# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""Remove the ground plane from a LiDAR cloud with RANSAC."""

import numpy as np
import open3d as o3d


def plane_tilt_deg(plane: np.ndarray) -> float:
    """Angle between the plane normal and vertical. 0 is a level plane."""
    normal = np.asarray(plane[:3], dtype=np.float64)
    length = float(np.linalg.norm(normal))
    if length == 0.0:
        return 90.0
    return float(np.degrees(np.arccos(min(abs(float(normal[2])) / length, 1.0))))


def remove_ground(
    pcd: o3d.geometry.PointCloud,
    ransac_dist: float,
    ransac_iters: int,
    max_tilt_deg: float,
    max_fits: int,
) -> o3d.geometry.PointCloud:
    """Return every point that is not part of the largest level plane in pcd.

    A plane tilted more than max_tilt_deg is a wall or similar, not the floor. It
    is set aside so the next fit can find the floor, and it stays in the output.
    If no level plane turns up in max_fits fits, nothing is removed.
    """
    search = pcd
    remaining = np.arange(len(pcd.points))  # index in pcd of each point in search
    for _ in range(max_fits):
        if len(search.points) < 50:  # not enough points to make a plane
            break
        plane, inliers = search.segment_plane(  # running ransac plane filtering
            distance_threshold=ransac_dist,  # how far away a point can be from plane
            ransac_n=3,  # each plane hypothesis is 3 sampled points
            num_iterations=ransac_iters,
        )
        if plane_tilt_deg(plane) <= max_tilt_deg:
            return pcd.select_by_index(
                remaining[inliers].tolist(), invert=True
            )  # Select all points except for the floor
        search = search.select_by_index(inliers, invert=True)
        remaining = np.delete(remaining, inliers)
    return pcd
