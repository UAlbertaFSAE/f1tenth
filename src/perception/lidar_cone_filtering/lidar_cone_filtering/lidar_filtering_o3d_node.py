#!/usr/bin/env python3
# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""LiDAR point-cloud filtering for cone candidates.

Takes the raw point cloud, removes the ground plane and out-of-range returns with Open3D,
clusters what is left, and publishes the cluster centroids that look cone-sized. This is
the LiDAR half of perception; the camera half is ``camera_detection``.
"""

from typing import cast

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from perception_observation.transport import ObservationPublisher, observation
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from lidar_cone_filtering.clustering import cluster_points
from lidar_cone_filtering.cone_classification import (
    cone_centroids,
    remove_isolated_cones,
    remove_tall_clusters,
)
from lidar_cone_filtering.ground_removal import remove_ground
from lidar_cone_filtering.preprocessing import (
    downsample_and_denoise,
    roi_crop,
)


class LidarFilteringOpen3D(Node):
    """ROS 2 node for Open3D-based LiDAR filtering and cone cluster centroids."""

    def __init__(self) -> None:
        """Declare parameters and create subscriptions and publishers."""
        super().__init__("lidar_filtering_o3d")

        # Parameters
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("no_ground_topic", "/lidar/no_ground")
        self.declare_parameter("centroids_topic", "/lidar/cone_cluster_centroids")

        self.declare_parameter("observation_topic", "/perception/lidar/observations")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("infer_color_from_heading", True)
        self.observation_output = ObservationPublisher(
            self,
            "lidar",
            self.get_parameter("observation_topic").value,
            self.get_parameter("target_frame").value,
        )

        # limiting roi
        self.declare_parameter("roi_x_min", 0.0)
        self.declare_parameter("roi_x_max", 5.0)
        self.declare_parameter("roi_y_min", -2.0)
        self.declare_parameter("roi_y_max", 2.0)
        self.declare_parameter("roi_z_min", -0.5)
        self.declare_parameter("roi_z_max", 0.5)

        # Cone geometry filtering, based on FSAE Driverless cone regulations
        self.declare_parameter("small_cone_height", 0.325)
        self.declare_parameter("large_cone_height", 0.505)
        self.declare_parameter("cone_height_tolerance", 0.10)
        self.declare_parameter("small_cone_width", 0.228)
        self.declare_parameter("large_cone_width", 0.285)
        self.declare_parameter("cone_width_tolerance", 0.10)
        self.declare_parameter("cone_min_height", 0.03)

        self.declare_parameter(
            "cone_neighbor_radius", 3.0
        )  # how close another cone must be to count as a neighbor
        self.declare_parameter(
            "cone_min_neighbors", 2
        )  # cones with fewer neighbors than this are removed, 0 turns the check off

        self.declare_parameter(
            "voxel", 0.01
        )  # voxel downsampling size, each cube will be 1cm^3,
        # all points in each cube get turned into a centroid
        self.declare_parameter(
            "sor_nb_neighbors", 20
        )  # look at 20 neighbors for each point
        self.declare_parameter(
            "sor_std_ratio", 2.0
        )  # remove points 2 SD from typical neighbor distance

        self.declare_parameter(
            "ransac_dist", 0.05
        )  # Points within 0.05m of the plane are counted as inliers
        self.declare_parameter(
            "ransac_iters", 1000
        )  # Do 1000 iterations to find best plane
        self.declare_parameter(
            "ground_max_tilt_deg", 15.0
        )  # Plane normal must be within this many degrees of vertical to be the floor
        self.declare_parameter(
            "ground_max_fits", 4
        )  # How many RANSAC fits to try before giving up on finding the floor

        # Open3D DBSCAN. With min_points=1
        self.declare_parameter(
            "cluster_eps", 0.25
        )  # points are considered part of same cluster  if they are within 0.25m of eachother
        self.declare_parameter(
            "cluster_min_points", 1
        )  # min # of points for a group of points to be considered a cluster
        self.declare_parameter("min_cluster_size", 10)
        self.declare_parameter("max_cluster_size", 2000)
        self.declare_parameter(
            "cluster_max_height_above_ground", 0.70
        )  # Reject a cluster whose highest point is more than this above the floor
        self.declare_parameter("marker_scale", 0.25)

        input_topic = self.get_parameter("input_topic").value
        no_ground_topic = self.get_parameter("no_ground_topic").value
        centroids_topic = self.get_parameter("centroids_topic").value

        # LiDAR topics often publish BEST_EFFORT
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos
        )
        self.pub_noground = self.create_publisher(PointCloud2, no_ground_topic, 5)
        self.pub_centroids = self.create_publisher(MarkerArray, centroids_topic, 5)

        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing no-ground: {no_ground_topic}")
        self.get_logger().info(f"Publishing centroids: {centroids_topic}")

    def callback(self, msg: PointCloud2) -> None:
        """Process one ``PointCloud2`` message and publish filtered output."""
        xyz = self.pc2_to_xyz(msg)
        if xyz.shape[0] < 50:
            self.observation_output.publish(msg.header, [])
            return

        x_min = float(self.get_parameter("roi_x_min").value)
        x_max = float(self.get_parameter("roi_x_max").value)
        y_min = float(self.get_parameter("roi_y_min").value)
        y_max = float(self.get_parameter("roi_y_max").value)
        z_min = float(self.get_parameter("roi_z_min").value)
        z_max = float(self.get_parameter("roi_z_max").value)

        xyz = roi_crop(xyz, x_min, x_max, y_min, y_max, z_min, z_max)
        if xyz.shape[0] < 50:
            self.observation_output.publish(msg.header, [])
            return

        objects_xyz, centroids = self.process_with_open3d(xyz)

        self.observation_output.publish(
            msg.header,
            [observation(msg.header, centroid) for centroid in centroids],
            infer_color=bool(self.get_parameter("infer_color_from_heading").value),
        )

        # Publish no-ground cloud
        self.pub_noground.publish(self.xyz_to_pc2(objects_xyz, msg.header))

        # Publish centroid markers
        self.pub_centroids.publish(self.centroids_to_markers(centroids, msg.header))

    def pc2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Convert ``msg`` to an ``(N, 3)`` float32 XYZ array."""
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if len(points) == 0:
            return np.empty((0, 3), dtype=np.float32)
        # Copy the fields by name. The Livox point_step is 26 bytes, so viewing
        # the padded records as float32 fails on older numpy.
        points_np = np.column_stack((points["x"], points["y"], points["z"]))
        return cast(np.ndarray, points_np.astype(np.float32))

    def xyz_to_pc2(self, xyz: np.ndarray, header: Header) -> PointCloud2:
        """Build a ``PointCloud2`` from XYZ rows using ``header``."""
        return pc2.create_cloud_xyz32(header, xyz.astype(np.float32).tolist())

    def process_with_open3d(
        self, xyz: np.ndarray
    ) -> tuple[np.ndarray, list[np.ndarray]]:
        """Downsample, remove outliers and ground, then cluster and compute centroids."""
        voxel = float(self.get_parameter("voxel").value)
        sor_nb = int(
            self.get_parameter("sor_nb_neighbors").value
        )  # number of neighbors used for outlier detection
        sor_std = float(
            self.get_parameter("sor_std_ratio").value
        )  # threshold for how far a point can be before getting removed

        ransac_dist = float(self.get_parameter("ransac_dist").value)
        ransac_iters = int(self.get_parameter("ransac_iters").value)
        max_tilt = float(self.get_parameter("ground_max_tilt_deg").value)
        max_fits = int(self.get_parameter("ground_max_fits").value)
        max_top = float(self.get_parameter("cluster_max_height_above_ground").value)
        cluster_eps = float(self.get_parameter("cluster_eps").value)
        cluster_min_pts = int(self.get_parameter("cluster_min_points").value)
        min_cluster = int(self.get_parameter("min_cluster_size").value)
        max_cluster = int(self.get_parameter("max_cluster_size").value)
        small_height = float(self.get_parameter("small_cone_height").value)
        large_height = float(self.get_parameter("large_cone_height").value)
        tolerance = float(self.get_parameter("cone_height_tolerance").value)
        small_width = float(self.get_parameter("small_cone_width").value)
        large_width = float(self.get_parameter("large_cone_width").value)
        width_tolerance = float(self.get_parameter("cone_width_tolerance").value)
        min_height = float(self.get_parameter("cone_min_height").value)
        neighbor_radius = float(self.get_parameter("cone_neighbor_radius").value)
        min_neighbors = int(self.get_parameter("cone_min_neighbors").value)
        pcd = downsample_and_denoise(xyz, voxel, sor_nb, sor_std)
        objects, floor = remove_ground(
            pcd, ransac_dist, ransac_iters, max_tilt, max_fits
        )

        objects_xyz = np.asarray(
            objects.points, dtype=np.float32
        )  # convert o3d object back to numpy

        clusters = cluster_points(
            objects, cluster_eps, cluster_min_pts, min_cluster, max_cluster
        )
        clusters = remove_tall_clusters(clusters, floor, max_top)

        centroids = cone_centroids(
            clusters,
            small_height,
            large_height,
            tolerance,
            small_width,
            large_width,
            width_tolerance,
            min_height,
        )
        centroids = remove_isolated_cones(centroids, neighbor_radius, min_neighbors)

        return objects_xyz, centroids

    def centroids_to_markers(
        self, centroids: list[np.ndarray], header: Header
    ) -> MarkerArray:
        """Publish RViz markers for cluster centroids."""
        marker_scale = float(self.get_parameter("marker_scale").value)

        ma = MarkerArray()

        # Clear old markers every frame (so RViz doesn't leave stale ones)
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, c in enumerate(centroids):
            mk = Marker()
            mk.header = header
            mk.ns = "cone_clusters"
            mk.id = i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD

            mk.pose.position.x = float(c[0])
            mk.pose.position.y = float(c[1])
            mk.pose.position.z = float(c[2])
            mk.pose.orientation.w = 1.0

            mk.scale.x = marker_scale
            mk.scale.y = marker_scale
            mk.scale.z = marker_scale

            mk.color.a = 1.0
            mk.color.r = 1.0
            mk.color.g = 0.5
            mk.color.b = 0.0

            ma.markers.append(mk)

        return ma


def main() -> None:
    """Run the node until shutdown."""
    rclpy.init()
    node = LidarFilteringOpen3D()
    # Keep TF callbacks serviced while Open3D processes a cloud.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
