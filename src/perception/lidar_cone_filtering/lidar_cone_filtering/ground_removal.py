# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""Remove the ground plane from a LiDAR cloud with RANSAC."""

import open3d as o3d


def remove_ground(
    pcd: o3d.geometry.PointCloud, ransac_dist: float, ransac_iters: int
) -> o3d.geometry.PointCloud:
    """Return every point that is not part of the largest plane in pcd."""
    if len(pcd.points) >= 50:
        _, inliers = pcd.segment_plane(  # running ransac plane filtering
            distance_threshold=ransac_dist,  # how far away a point can be from plane
            ransac_n=3,  # each plane hypothesis is 3 sampled points
            num_iterations=ransac_iters,
        )
        objects = pcd.select_by_index(
            inliers, invert=True
        )  # Select all points except for the plane
    else:  # not enough points to make a plane
        objects = pcd
    return objects
