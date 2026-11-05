# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""Crop, downsample and remove outliers from a LiDAR scan."""

from typing import cast

import numpy as np
import open3d as o3d


def roi_crop(
    xyz: np.ndarray,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> np.ndarray:
    """Keep points inside the configured ROI box."""
    m = (
        (xyz[:, 0] >= x_min)
        & (xyz[:, 0] <= x_max)
        & (xyz[:, 1] >= y_min)
        & (xyz[:, 1] <= y_max)
        & (xyz[:, 2] >= z_min)
        & (xyz[:, 2] <= z_max)
    )
    return cast(np.ndarray, xyz[m])


def downsample_and_denoise(
    xyz: np.ndarray, voxel: float, sor_nb: int, sor_std: float
) -> o3d.geometry.PointCloud:
    """Turn the points into an Open3D cloud, downsample it and remove outliers."""
    pcd = o3d.geometry.PointCloud()  # creating open3d point cloud object
    pcd.points = o3d.utility.Vector3dVector(
        xyz.astype(np.float64)
    )  # populating pcd.points with the xyz pc data

    if voxel > 0.0:
        pcd = pcd.voxel_down_sample(
            voxel_size=voxel
        )  # thins out data points, one point per voxel^3 cube

    if len(pcd.points) > sor_nb:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=sor_nb, std_ratio=sor_std
        )  # remove points that are sor_nb away from normal
    return pcd
