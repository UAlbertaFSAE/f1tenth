#!/usr/bin/env python3
from typing import cast

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


class LidarFilteringOpen3D(Node):
    """ROS 2 node for Open3D-based LiDAR filtering and cone cluster centroids."""

    def __init__(self) -> None:
        """Declare parameters and create subscriptions and publishers."""
        super().__init__("lidar_filtering_o3d")

        # Parameters
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("no_ground_topic", "/lidar/no_ground")
        self.declare_parameter("centroids_topic", "/lidar/cone_cluster_centroids")

        #limiting roi
        self.declare_parameter("roi_x_min", 0.0)
        self.declare_parameter("roi_x_max", 30.0)
        self.declare_parameter("roi_y_min", -10.0)
        self.declare_parameter("roi_y_max", 10.0)
        self.declare_parameter("roi_z_min", -2.0)
        self.declare_parameter("roi_z_max", 2.0)

        self.declare_parameter("voxel", 0.05) #voxel downsampling size, each cube will be 5cm^3, all points in each cube get turned into a centroid
        self.declare_parameter("sor_nb_neighbors", 20) #look at 20 neighbors for each point
        self.declare_parameter("sor_std_ratio", 2.0) #remove points 2 SD from typical neighbor distance

        self.declare_parameter("ransac_dist", 0.05) #Points within 0.05m of the plane are counted as inliers
        self.declare_parameter("ransac_iters", 1000) #Do 1000 iterations to find best plane

        # Open3D DBSCAN. With min_points=1
        self.declare_parameter("cluster_eps", 0.25) #points are considered part of same cluster  if they are within 0.25m of eachother
        self.declare_parameter("cluster_min_points", 1) #min # of points for a group of points to be considered a cluster
        self.declare_parameter("min_cluster_size", 10) 
        self.declare_parameter("max_cluster_size", 2000)

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

        self.sub = self.create_subscription(PointCloud2, input_topic, self.callback, qos)
        self.pub_noground = self.create_publisher(PointCloud2, no_ground_topic, 5)
        self.pub_centroids = self.create_publisher(MarkerArray, centroids_topic, 5)

        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing no-ground: {no_ground_topic}")
        self.get_logger().info(f"Publishing centroids: {centroids_topic}")

    def callback(self, msg: PointCloud2) -> None:
        """Process one ``PointCloud2`` message and publish filtered output."""
        xyz = self.pc2_to_xyz(msg)
        if xyz.shape[0] < 50:
            return

        xyz = self.roi_crop(xyz)
        if xyz.shape[0] < 50:
            return

        objects_xyz, centroids = self.process_with_open3d(xyz)

        # Publish no-ground cloud
        self.pub_noground.publish(self.xyz_to_pc2(objects_xyz, msg.header))

        # Publish centroid markers
        self.pub_centroids.publish(self.centroids_to_markers(centroids, msg.header))

    def pc2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Convert ``msg`` to an ``(N, 3)`` float32 XYZ array."""
        pts = np.array(
            list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32,
        )
        return cast(np.ndarray, pts)

    def xyz_to_pc2(self, xyz: np.ndarray, header: Header) -> PointCloud2:
        """Build a ``PointCloud2`` from XYZ rows using ``header``."""
        return point_cloud2.create_cloud_xyz32(header, xyz.astype(np.float32).tolist())

    def roi_crop(self, xyz: np.ndarray) -> np.ndarray:
        """Keep points inside the configured ROI box."""
        x_min = float(self.get_parameter("roi_x_min").value)
        x_max = float(self.get_parameter("roi_x_max").value)
        y_min = float(self.get_parameter("roi_y_min").value)
        y_max = float(self.get_parameter("roi_y_max").value)
        z_min = float(self.get_parameter("roi_z_min").value)
        z_max = float(self.get_parameter("roi_z_max").value)

        m = (
            (xyz[:, 0] >= x_min) & (xyz[:, 0] <= x_max) &
            (xyz[:, 1] >= y_min) & (xyz[:, 1] <= y_max) &
            (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
        )
        return cast(np.ndarray, xyz[m])

    def process_with_open3d(self, xyz: np.ndarray) -> tuple[np.ndarray, list[np.ndarray]]:
        """Downsample, remove outliers and ground, then cluster and compute centroids."""
        voxel = float(self.get_parameter("voxel").value)
        sor_nb = int(self.get_parameter("sor_nb_neighbors").value) #number of neighbors used for outlier detection
        sor_std = float(self.get_parameter("sor_std_ratio").value) #threshold for how far a point can be before getting removed

        ransac_dist = float(self.get_parameter("ransac_dist").value)
        ransac_iters = int(self.get_parameter("ransac_iters").value)

        cluster_eps = float(self.get_parameter("cluster_eps").value)
        cluster_min_pts = int(self.get_parameter("cluster_min_points").value)
        min_cluster = int(self.get_parameter("min_cluster_size").value)
        max_cluster = int(self.get_parameter("max_cluster_size").value)

        pcd = o3d.geometry.PointCloud() #creating open3d point cloud object
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64)) #populating pcd.points with the xyz pc data

        if voxel > 0.0:
            pcd = pcd.voxel_down_sample(voxel_size=voxel) #thins out data points, one point per voxel^3 cube

        if len(pcd.points) > sor_nb:
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=sor_nb, std_ratio=sor_std) #remove points that are sor_nb away from normal

        if len(pcd.points) >= 50:
            _, inliers = pcd.segment_plane( #running ransac plane filtering
                distance_threshold=ransac_dist, #how far away a point can be from plane
                ransac_n=3, #each plane hypothesis is 3 sampled points
                num_iterations=ransac_iters,
            )
            objects = pcd.select_by_index(inliers, invert=True) #Select all points except for the plane
        else: #not enough points to make a plane
            objects = pcd

        objects_xyz = np.asarray(objects.points, dtype=np.float32) #convert o3d object back to numpy

        centroids: list[np.ndarray] = [] #holds one [x, y, z] centroid per accepted cluster
        if len(objects.points) > 0:
            labels = np.array(objects.cluster_dbscan(
                eps=cluster_eps,
                min_points=cluster_min_pts,
                print_progress=False
            ), dtype=np.int32)

            if labels.size > 0 and labels.max() >= 0:
                for k in range(labels.max() + 1):
                    cluster_pts = objects_xyz[labels == k]
                    n = cluster_pts.shape[0]
                    if min_cluster <= n <= max_cluster:
                        centroids.append(cluster_pts.mean(axis=0))

        return objects_xyz, centroids

    def centroids_to_markers(self, centroids: list[np.ndarray], header: Header) -> MarkerArray:
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
