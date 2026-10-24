#!/usr/bin/env python3
# Copyright (c) 2026 UAlberta Formula SAE
#
# Licensed under the MIT License. See the LICENSE file in this package, or the
# one at the repository root, for the full text.

"""LiDAR point-cloud filtering for cone candidates.

Takes the raw point cloud, removes the ground plane and out-of-range returns with Open3D,
clusters what is left, and publishes the cluster centroids that look cone-sized. A short
stack of scans is aligned in the odom frame first, so a cone that is only a few points
in one scan can still form a cluster. This is the LiDAR half of perception; the camera
half is ``camera_detection``.
"""

from collections import deque
from typing import cast

import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
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

        # limiting roi
        self.declare_parameter("roi_x_min", 0.0)
        self.declare_parameter("roi_x_max", 30.0)
        self.declare_parameter("roi_y_min", -10.0)
        self.declare_parameter("roi_y_max", 10.0)
        self.declare_parameter("roi_z_min", -2.0)
        self.declare_parameter("roi_z_max", 2.0)

        # Cone geometry filtering, based on FSAE Driverless cone regulations
        self.declare_parameter("small_cone_height", 0.325)
        self.declare_parameter("large_cone_height", 0.505)
        self.declare_parameter("cone_height_tolerance", 0.10)
        self.declare_parameter("small_cone_width", 0.228)
        self.declare_parameter("large_cone_width", 0.285)
        self.declare_parameter("cone_width_tolerance", 0.10)

        self.declare_parameter(
            "voxel", 0.05
        )  # voxel downsampling size, each cube will be 5cm^3, all points in each cube get turned into a centroid
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
        )  # Plane normal must be within this many degrees of vertical
        self.declare_parameter(
            "ground_max_z", 0.15
        )  # Median inlier height must be at or below this, so a ceiling is not the floor
        self.declare_parameter(
            "plane_max_fits", 8
        )  # How many RANSAC fits to try while peeling off the floor and walls
        self.declare_parameter(
            "wall_min_tilt_deg", 75.0
        )  # Normal must be at least this far from vertical; 90 degrees is a vertical wall
        self.declare_parameter(
            "wall_min_points", 200
        )  # Ignore vertical planes smaller than this, so a cone is not deleted as a wall
        self.declare_parameter(
            "wall_min_width", 2.0
        )  # Inliers must span at least this many metres horizontally
        self.declare_parameter(
            "wall_min_height", 1.0
        )  # Inliers must span at least this many metres vertically
        self.declare_parameter(
            "plane_fit_attempts", 3
        )  # Repeat each fit and keep the largest plane that qualifies

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

        # Align this many seconds of scans in accumulate_frame before filtering.
        # Each scan is voxelized on its own, so repeated hits stay separate points.
        # 0 clusters the current scan only.
        self.declare_parameter("accumulate_window_sec", 0.30)
        self.declare_parameter("accumulate_frame", "odom")

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

        # (stamp_ns, voxelized points in accumulate_frame)
        self._scan_stack: deque[tuple[int, np.ndarray]] = deque()
        self._last_clock_ns: int | None = None
        self._last_cloud_ns: int | None = None
        self._static_tfs: dict[tuple[str, str], object] = {}
        accumulate_window = float(self.get_parameter("accumulate_window_sec").value)
        accumulate_frame = str(self.get_parameter("accumulate_frame").value)
        self.tf_buffer: Buffer | None = None
        self.tf_listener: TransformListener | None = None
        if accumulate_window > 0.0:
            self.tf_buffer = Buffer()
            # /tf has to be applied while a scan is filtering. One executor
            # leaves the pose about 100 ms behind, and that smears cones once
            # the car is moving.
            self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)
            static_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
            )
            self.create_subscription(
                TFMessage, "/tf_static", self._on_tf_static, static_qos
            )
            clock_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(Clock, "/clock", self._on_clock, clock_qos)
            self.get_logger().info(
                f"Accumulating {accumulate_window:.2f}s of scans in {accumulate_frame}"
            )

    def is_cone_cluster(self, cluster_pts: np.ndarray) -> bool:
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

        small_height = float(self.get_parameter("small_cone_height").value)
        large_height = float(self.get_parameter("large_cone_height").value)
        tolerance = float(self.get_parameter("cone_height_tolerance").value)

        small_width = float(self.get_parameter("small_cone_width").value)

        large_width = float(self.get_parameter("large_cone_width").value)

        width_tolerance = float(self.get_parameter("cone_width_tolerance").value)

        small_ok = (
            abs(height - small_height) <= tolerance
            and abs(width - small_width) <= width_tolerance
        )

        large_ok = (
            abs(height - large_height) <= tolerance
            and abs(width - large_width) <= width_tolerance
        )

        return small_ok or large_ok

    def callback(self, msg: PointCloud2) -> None:
        """Process one ``PointCloud2`` message and publish filtered output."""
        xyz = self.pc2_to_xyz(msg)
        if xyz.shape[0] < 50:
            return

        xyz = self.roi_crop(xyz)
        if xyz.shape[0] < 50:
            return

        stripped, ground_plane = self._strip_scan(xyz)
        stacked, _ = self._aligned_stack(stripped, msg.header)
        objects_xyz, centroids = self.process_with_open3d(
            stacked, downsample=False, ground_plane=ground_plane
        )

        # Publish no-ground cloud
        self.pub_noground.publish(self.xyz_to_pc2(objects_xyz, msg.header))

        # Publish centroid markers
        self.pub_centroids.publish(self.centroids_to_markers(centroids, msg.header))

    def pc2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Convert ``msg`` to an ``(N, 3)`` float32 XYZ array."""
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = list(gen)
        if not points:
            return cast(np.ndarray, np.empty((0, 3), dtype=np.float32))
        points_np = np.array(points).view(np.float32).reshape(-1, 3)
        return cast(np.ndarray, points_np)

    def xyz_to_pc2(self, xyz: np.ndarray, header: Header) -> PointCloud2:
        """Build a ``PointCloud2`` from XYZ rows using ``header``."""
        return pc2.create_cloud_xyz32(header, xyz.astype(np.float32).tolist())

    def roi_crop(self, xyz: np.ndarray) -> np.ndarray:
        """Keep points inside the configured ROI box."""
        x_min = float(self.get_parameter("roi_x_min").value)
        x_max = float(self.get_parameter("roi_x_max").value)
        y_min = float(self.get_parameter("roi_y_min").value)
        y_max = float(self.get_parameter("roi_y_max").value)
        z_min = float(self.get_parameter("roi_z_min").value)
        z_max = float(self.get_parameter("roi_z_max").value)

        m = (
            (xyz[:, 0] >= x_min)
            & (xyz[:, 0] <= x_max)
            & (xyz[:, 1] >= y_min)
            & (xyz[:, 1] <= y_max)
            & (xyz[:, 2] >= z_min)
            & (xyz[:, 2] <= z_max)
        )
        return cast(np.ndarray, xyz[m])

    def _aligned_stack(
        self, xyz: np.ndarray, header: Header
    ) -> tuple[np.ndarray, bool]:
        """Keep the floor and walls off one scan, then append it to the history.

        Planes are removed before stacking so one bad fit cannot delete the cones
        from the other scans. The returned cloud is in the scan's own frame. The
        bool is unused by the caller; the cloud is already voxelized. A missing
        transform falls back to this scan alone.
        """
        window = float(self.get_parameter("accumulate_window_sec").value)
        if window <= 0.0 or self.tf_buffer is None:
            return xyz, False

        source_frame = header.frame_id or "livox_frame"
        target_frame = str(self.get_parameter("accumulate_frame").value)
        stamp = Time.from_msg(header.stamp)
        self._note_rewind(stamp.nanoseconds, "_last_cloud_ns")
        aligned = self._transform_xyz(xyz, source_frame, target_frame, stamp)
        if aligned is None:
            return xyz, False

        stamp_ns = stamp.nanoseconds
        if self._scan_stack and stamp_ns + 50_000_000 < self._scan_stack[-1][0]:
            self._scan_stack.clear()
        self._scan_stack.append((stamp_ns, aligned))
        cutoff_ns = stamp_ns - int(window * 1_000_000_000)
        while self._scan_stack and self._scan_stack[0][0] < cutoff_ns:
            self._scan_stack.popleft()

        stacked = np.concatenate([points for _, points in self._scan_stack], axis=0)
        in_sensor = self._transform_xyz(stacked, target_frame, source_frame, stamp)
        if in_sensor is None:
            return xyz, False
        return in_sensor, True

    def _voxel_downsample(self, xyz: np.ndarray) -> np.ndarray:
        """Collapse ``xyz`` to one point per configured voxel cube."""
        voxel = float(self.get_parameter("voxel").value)
        if voxel <= 0.0 or len(xyz) == 0:
            return xyz
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
        pcd = pcd.voxel_down_sample(voxel_size=voxel)
        return np.asarray(pcd.points, dtype=np.float32)

    def _transform_xyz(
        self,
        xyz: np.ndarray,
        source_frame: str,
        target_frame: str,
        stamp: Time,
    ) -> np.ndarray | None:
        """Apply the transform that takes points from ``source_frame`` to ``target_frame``."""
        if self.tf_buffer is None or len(xyz) == 0:
            return None if self.tf_buffer is None else xyz
        transform = self._lookup_transform(target_frame, source_frame, stamp)
        if transform is None:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rot = self._quat_to_rot(rotation.x, rotation.y, rotation.z, rotation.w)
        offset = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float64
        )
        transformed = xyz.astype(np.float64) @ rot.T + offset
        return transformed.astype(np.float32)

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp: Time):
        """Return the transform at ``stamp``.

        A bag played on loop publishes transforms from the previous pass first.
        Those are newer than the restarted clock, so the buffer keeps them and
        refuses every later sample. Stacking with that frozen pose smears cones
        as soon as the car moves. Drop it and wait for a transform near this scan.
        """
        if self.tf_buffer is None:
            return None
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=0.0)
            )
        except TransformException as ex:
            try:
                latest = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, Time()
                )
            except TransformException as fallback_ex:
                self.get_logger().warn(
                    f"TF {source_frame} -> {target_frame} failed: {ex} "
                    f"(latest also failed: {fallback_ex})",
                    throttle_duration_sec=5.0,
                )
                return None
            latest_ns = Time.from_msg(latest.header.stamp).nanoseconds
            gap_ns = abs(latest_ns - stamp.nanoseconds)
            # A bag loop leaves the previous pass's pose seconds away from this
            # scan. A fraction of a second is just the newest sample and must
            # not wipe the stack.
            if gap_ns > 2_000_000_000:
                self._reset_motion_state()
                self.get_logger().warn(
                    f"TF {source_frame} -> {target_frame} is {gap_ns / 1e9:.1f}s "
                    "away from this scan, so the scan was not added to the stack.",
                    throttle_duration_sec=5.0,
                )
                return None
            if gap_ns > 50_000_000:
                self.get_logger().warn(
                    f"TF {source_frame} -> {target_frame} is {gap_ns / 1e9:.3f}s "
                    "from this scan; stacking this scan was skipped.",
                    throttle_duration_sec=5.0,
                )
                return None
            return latest

    def _on_clock(self, msg: Clock) -> None:
        """Clear motion history when sim time jumps backwards, as on a bag loop."""
        self._note_rewind(Time.from_msg(msg.clock).nanoseconds, "_last_clock_ns")

    def _on_tf_static(self, msg: TFMessage) -> None:
        """Remember static transforms so a buffer reset can restore them."""
        for transform in msg.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            self._static_tfs[key] = transform

    def _note_rewind(self, stamp_ns: int, slot: str) -> None:
        """Reset alignment when one time source jumps backwards."""
        previous = getattr(self, slot)
        setattr(self, slot, stamp_ns)
        if previous is not None and stamp_ns + 500_000_000 < previous:
            self._reset_motion_state()

    def _reset_motion_state(self) -> None:
        """Drop the scan stack and the transform cache, then restore static frames."""
        self._scan_stack.clear()
        if self.tf_buffer is None:
            return
        self.tf_buffer.clear()
        for transform in self._static_tfs.values():
            self.tf_buffer.set_transform_static(transform, "default_authority")
        self.get_logger().info(
            "Cleared the scan stack and transform buffer after a time jump.",
            throttle_duration_sec=5.0,
        )

    def _quat_to_rot(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """Rotation matrix for a quaternion in xyzw order."""
        norm = float(np.sqrt(x * x + y * y + z * z + w * w))
        if norm == 0.0:
            return np.eye(3, dtype=np.float64)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

    def remove_structural_planes(
        self,
        pcd: o3d.geometry.PointCloud,
        ransac_dist: float,
        ransac_iters: int,
    ) -> tuple[o3d.geometry.PointCloud, np.ndarray | None]:
        """Remove the floor and walls from one voxelized scan.

        ``segment_plane`` is run several times and the largest plane that is
        either the floor or a wall is kept. A wall has to be near-vertical and
        span ``wall_min_width`` by ``wall_min_height``, so a cone-sized patch
        does not qualify. Both are deleted. A ceiling or table is neither, so
        it is skipped and the search continues. The floor plane is returned for
        the height check.
        """
        max_tilt_deg = float(self.get_parameter("ground_max_tilt_deg").value)
        ground_max_z = float(self.get_parameter("ground_max_z").value)
        max_fits = int(self.get_parameter("plane_max_fits").value)
        attempts = int(self.get_parameter("plane_fit_attempts").value)
        wall_min_tilt_deg = float(self.get_parameter("wall_min_tilt_deg").value)
        wall_min_points = int(self.get_parameter("wall_min_points").value)
        wall_min_width = float(self.get_parameter("wall_min_width").value)
        wall_min_height = float(self.get_parameter("wall_min_height").value)

        search = pcd
        remove_indices: set[int] = set()
        ground_plane: np.ndarray | None = None
        for _ in range(max_fits):
            if len(search.points) < 50:
                break
            plane, inliers, is_ground, is_wall = self._choose_plane(
                search,
                ransac_dist,
                ransac_iters,
                attempts,
                max_tilt_deg,
                ground_max_z,
                wall_min_tilt_deg,
                wall_min_points,
                wall_min_width,
                wall_min_height,
            )
            if plane is None or len(inliers) < 50:
                break
            if is_ground or is_wall:
                remove_indices.update(self._indices_near_plane(pcd, plane, ransac_dist))
            elif len(inliers) < wall_min_points:
                break
            if is_ground and ground_plane is None:
                ground_plane = np.asarray(plane, dtype=np.float64)
            search = search.select_by_index(inliers, invert=True)
        if not remove_indices:
            return pcd, ground_plane
        return pcd.select_by_index(list(remove_indices), invert=True), ground_plane

    def _strip_scan(self, xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray | None]:
        """Voxelize one scan and remove its floor and walls."""
        voxel_xyz = self._voxel_downsample(xyz)
        if len(voxel_xyz) < 50:
            return voxel_xyz, None
        ransac_dist = float(self.get_parameter("ransac_dist").value)
        ransac_iters = int(self.get_parameter("ransac_iters").value)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(voxel_xyz.astype(np.float64))
        objects, ground_plane = self.remove_structural_planes(
            pcd, ransac_dist, ransac_iters
        )
        return np.asarray(objects.points, dtype=np.float32), ground_plane

    def _choose_plane(
        self,
        search: o3d.geometry.PointCloud,
        ransac_dist: float,
        ransac_iters: int,
        attempts: int,
        max_tilt_deg: float,
        ground_max_z: float,
        wall_min_tilt_deg: float,
        wall_min_points: int,
        wall_min_width: float,
        wall_min_height: float,
    ) -> tuple[np.ndarray | None, list[int], bool, bool]:
        """Pick the largest floor or wall fit, otherwise the largest plane."""
        best: tuple[int, np.ndarray, list[int], bool, bool] | None = None
        largest: tuple[int, np.ndarray, list[int]] | None = None
        for _ in range(max(attempts, 1)):
            plane, inliers = search.segment_plane(
                distance_threshold=ransac_dist,
                ransac_n=3,
                num_iterations=ransac_iters,
            )
            count = len(inliers)
            if largest is None or count > largest[0]:
                largest = (count, plane, inliers)
            is_ground = self._is_ground_plane(
                plane, search, inliers, max_tilt_deg, ground_max_z
            )
            is_wall = self._is_wall_plane(
                plane,
                search,
                inliers,
                wall_min_tilt_deg,
                wall_min_points,
                wall_min_width,
                wall_min_height,
            )
            if (is_ground or is_wall) and (best is None or count > best[0]):
                best = (count, plane, inliers, is_ground, is_wall)
        if best is not None:
            return best[1], best[2], best[3], best[4]
        if largest is None:
            return None, [], False, False
        return largest[1], largest[2], False, False

    def _is_ground_plane(
        self,
        plane: np.ndarray,
        cloud: o3d.geometry.PointCloud,
        inliers: list[int],
        max_tilt_deg: float,
        ground_max_z: float,
    ) -> bool:
        """True when ``plane`` is near-horizontal and its inliers sit below the sensor."""
        if self._tilt_from_up_deg(plane) > max_tilt_deg:
            return False
        inlier_z = np.asarray(cloud.points)[inliers, 2]
        return float(np.median(inlier_z)) <= ground_max_z

    def _is_wall_plane(
        self,
        plane: np.ndarray,
        cloud: o3d.geometry.PointCloud,
        inliers: list[int],
        min_tilt_deg: float,
        min_points: int,
        min_width: float,
        min_height: float,
    ) -> bool:
        """True when ``plane`` is a tall, wide, near-vertical surface."""
        if len(inliers) < min_points:
            return False
        if self._tilt_from_up_deg(plane) < min_tilt_deg:
            return False
        points = np.asarray(cloud.points)[inliers]
        horizontal = float(np.hypot(np.ptp(points[:, 0]), np.ptp(points[:, 1])))
        vertical = float(np.ptp(points[:, 2]))
        return horizontal >= min_width and vertical >= min_height

    def _tilt_from_up_deg(self, plane: np.ndarray) -> float:
        """Angle between the plane normal and the z axis. 0 is a level floor."""
        a, b, c, _ = (float(value) for value in plane)
        norm = float(np.sqrt(a * a + b * b + c * c))
        if norm == 0.0:
            return 0.0
        return float(np.degrees(np.arccos(np.clip(abs(c) / norm, 0.0, 1.0))))

    def _indices_near_plane(
        self,
        cloud: o3d.geometry.PointCloud,
        plane: np.ndarray,
        distance: float,
    ) -> list[int]:
        """Indices of points in ``cloud`` within ``distance`` metres of ``plane``."""
        points = np.asarray(cloud.points)
        a, b, c, d = (float(value) for value in plane)
        norm = float(np.sqrt(a * a + b * b + c * c))
        if norm == 0.0 or len(points) == 0:
            return []
        distances = np.abs(points @ np.array([a, b, c]) + d) / norm
        return cast(list[int], np.flatnonzero(distances <= distance).tolist())

    def _heights_above_ground(
        self, points: np.ndarray, plane: np.ndarray
    ) -> np.ndarray:
        """Signed height of each point above ``plane``. Positive is above the floor."""
        a, b, c, d = (float(value) for value in plane)
        if c < 0.0:
            a, b, c, d = -a, -b, -c, -d
        norm = float(np.sqrt(a * a + b * b + c * c))
        if norm == 0.0 or len(points) == 0:
            return np.zeros(len(points), dtype=np.float64)
        return cast(np.ndarray, (points @ np.array([a, b, c]) + d) / norm)

    def process_with_open3d(
        self,
        xyz: np.ndarray,
        downsample: bool = True,
        ground_plane: np.ndarray | None = None,
    ) -> tuple[np.ndarray, list[np.ndarray]]:
        """Remove outliers and cluster.

        The floor and walls are already gone. ``ground_plane`` is the floor of
        the current scan, in the same frame as ``xyz``. ``downsample`` stays
        available for a cloud that has not been voxelized yet.
        """
        voxel = float(self.get_parameter("voxel").value)
        sor_nb = int(
            self.get_parameter("sor_nb_neighbors").value
        )  # number of neighbors used for outlier detection
        sor_std = float(
            self.get_parameter("sor_std_ratio").value
        )  # threshold for how far a point can be before getting removed

        cluster_eps = float(self.get_parameter("cluster_eps").value)
        cluster_min_pts = int(self.get_parameter("cluster_min_points").value)
        min_cluster = int(self.get_parameter("min_cluster_size").value)
        max_cluster = int(self.get_parameter("max_cluster_size").value)
        max_height = float(self.get_parameter("cluster_max_height_above_ground").value)

        pcd = o3d.geometry.PointCloud()  # creating open3d point cloud object
        pcd.points = o3d.utility.Vector3dVector(
            xyz.astype(np.float64)
        )  # populating pcd.points with the xyz pc data

        if downsample and voxel > 0.0:
            pcd = pcd.voxel_down_sample(
                voxel_size=voxel
            )  # thins out data points, one point per voxel^3 cube

        if len(pcd.points) > sor_nb:
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=sor_nb, std_ratio=sor_std
            )  # remove points that are sor_nb away from normal

        objects = pcd

        objects_xyz = np.asarray(
            objects.points, dtype=np.float32
        )  # convert o3d object back to numpy

        centroids: list[
            np.ndarray
        ] = []  # holds one [x, y, z] centroid per accepted cluster
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

                    if not self.is_cone_cluster(cluster_pts):
                        continue

                    if ground_plane is not None:
                        top = float(
                            np.max(
                                self._heights_above_ground(cluster_pts, ground_plane)
                            )
                        )
                        if top > max_height:
                            continue

                    centroids.append(cluster_pts.mean(axis=0))

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
