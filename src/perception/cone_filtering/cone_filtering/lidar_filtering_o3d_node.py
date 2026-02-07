#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


class LidarFilteringOpen3D(Node):
    def __init__(self):
        super().__init__("lidar_filtering_o3d")

        # --- Parameters (so you can tune without editing code) ---
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("no_ground_topic", "/lidar/no_ground")
        self.declare_parameter("centroids_topic", "/lidar/cone_cluster_centroids")

        self.declare_parameter("roi_x_min", 0.0)
        self.declare_parameter("roi_x_max", 30.0)
        self.declare_parameter("roi_y_min", -10.0)
        self.declare_parameter("roi_y_max", 10.0)
        self.declare_parameter("roi_z_min", -2.0)
        self.declare_parameter("roi_z_max", 2.0)

        self.declare_parameter("voxel", 0.05)
        self.declare_parameter("sor_nb_neighbors", 20)
        self.declare_parameter("sor_std_ratio", 2.0)

        self.declare_parameter("ransac_dist", 0.05)
        self.declare_parameter("ransac_iters", 1000)

        # Open3D DBSCAN. With min_points=1 it's very "Euclidean-like".
        self.declare_parameter("cluster_eps", 0.25)
        self.declare_parameter("cluster_min_points", 1)
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

    def callback(self, msg: PointCloud2):
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
        # TODO: Convert to np.fromiter for better performance.
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = list(gen)
        if not points:
            return np.empty((0, 3), dtype=np.float32)
        points_np = np.array(points).view(np.float32).reshape(-1, 3)
        return points_np

    def xyz_to_pc2(self, xyz: np.ndarray, header) -> PointCloud2:
        return pc2.create_cloud_xyz32(header, xyz.astype(np.float32).tolist())

    def roi_crop(self, xyz: np.ndarray) -> np.ndarray:
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
        return xyz[m]

    def process_with_open3d(self, xyz: np.ndarray):
        voxel = float(self.get_parameter("voxel").value)
        sor_nb = int(self.get_parameter("sor_nb_neighbors").value)
        sor_std = float(self.get_parameter("sor_std_ratio").value)

        ransac_dist = float(self.get_parameter("ransac_dist").value)
        ransac_iters = int(self.get_parameter("ransac_iters").value)

        cluster_eps = float(self.get_parameter("cluster_eps").value)
        cluster_min_pts = int(self.get_parameter("cluster_min_points").value)
        min_cluster = int(self.get_parameter("min_cluster_size").value)
        max_cluster = int(self.get_parameter("max_cluster_size").value)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))

        if voxel > 0.0:
            pcd = pcd.voxel_down_sample(voxel_size=voxel)

        if len(pcd.points) > sor_nb:
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=sor_nb, std_ratio=sor_std)

        if len(pcd.points) >= 50:
            _, inliers = pcd.segment_plane(
                distance_threshold=ransac_dist,
                ransac_n=3,
                num_iterations=ransac_iters,
            )
            objects = pcd.select_by_index(inliers, invert=True)
        else:
            objects = pcd

        objects_xyz = np.asarray(objects.points, dtype=np.float32)

        centroids = []
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

    def centroids_to_markers(self, centroids, header) -> MarkerArray:
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


def main():
    rclpy.init()
    node = LidarFilteringOpen3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
