# import tf2_ros
import time

import numpy as np
import tf_transformations
from geometry_msgs.msg import (
    Pose,
    Quaternion,
)
from nav_msgs.msg import MapMetaData


class CircularArray:
    """Simple implementation of a circular array.

    You can append to it any number of times but only "size" items are kept.
    """

    def __init__(self, size: int) -> None:
        """Allocate a buffer holding the last ``size`` appended values."""
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value: float) -> None:
        """Append a value, overwriting the oldest one once the buffer is full."""
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    def mean(self) -> np.floating:
        """Return the mean of the values currently held."""
        return np.mean(self.arr[: self.num_els])

    def median(self) -> np.floating:
        """Return the median of the values currently held."""
        return np.median(self.arr[: self.num_els])


class Timer:
    """Simple helper class to compute the rate at which something is called.

    "smoothing" determines the size of the underlying circular array, which averages
    out variations in call rate over time.

    use timer.tick() to record an event
    use timer.fps() to report the average event rate.
    """

    def __init__(self, smoothing: int) -> None:
        """Average the call rate over the last ``smoothing`` events."""
        self.arr = CircularArray(smoothing)
        self.last_time = time.time()

    def tick(self) -> None:
        """Record an event."""
        t = time.time()
        self.arr.append(1.0 / (t - self.last_time))
        self.last_time = t

    def fps(self) -> np.floating:
        """Return the average event rate, in events per second."""
        return self.arr.mean()


def angle_to_quaternion(angle: float) -> Quaternion:
    """Convert an angle in radians into a quaternion _message_."""
    q = tf_transformations.quaternion_from_euler(0, 0, angle)
    q_out = Quaternion()
    q_out.x = q[0]
    q_out.y = q[1]
    q_out.z = q[2]
    q_out.w = q[3]
    return q_out


def quaternion_to_angle(q: Quaternion) -> float:
    """Convert a quaternion _message_ into an angle in radians.

    The angle represents the yaw. This is not just the z component of the quaternion.
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf_transformations.euler_from_quaternion((x, y, z, w))
    return float(yaw)


def rotation_matrix(theta: float) -> np.matrix:
    """Create a rotation matrix for the given angle in radians."""
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])


def particle_to_pose(particle: np.ndarray) -> Pose:
    """Convert a particle in the form [x, y, theta] into a Pose object."""
    pose = Pose()
    pose.position.x = particle[0]
    pose.position.y = particle[1]
    pose.orientation = angle_to_quaternion(float(particle[2]))
    return pose


def particles_to_poses(particles: np.ndarray) -> list[Pose]:
    """Convert a two dimensional array of particles into an array of Poses.

    Particles can be an array like [[x0, y0, theta0], [x1, y1, theta1], ...].
    """
    return list(map(particle_to_pose, particles))


# DEPRECATED: should make the header inside the node now
# def make_header(frame_id, stamp=None):
#     ''' Creates a Header object for stamped ROS objects '''
#     if stamp == None:
#         stamp = rospy.Time.now()
#     header = Header()
#     header.stamp = stamp
#     header.frame_id = frame_id
#     return header


def map_to_world_slow(
    x: float, y: float, t: float, map_info: MapMetaData
) -> tuple[float, float, float]:
    """Convert (x, y, t) from map coordinates (pixels) into world coordinates (meters).

    Provide the MapMetaData object from a map message to specify the change in
    coordinates. This is the logical but slow implementation: for a lot of coordinate
    conversions, use ``map_to_world`` instead.
    """
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)
    rot = rotation_matrix(angle)
    trans = np.array([[map_info.origin.position.x], [map_info.origin.position.y]])

    map_c = np.array([[x], [y]])
    world = (rot * map_c) * scale + trans

    return world[0, 0], world[1, 0], t + angle


def map_to_world(poses: np.ndarray, map_info: MapMetaData) -> None:
    """Convert poses from map coordinate space (pixels) to world coordinate space (meters).

    ``poses`` is a two dimensional numpy array::

        [[x0, y0, theta0],
         [x1, y1, theta1],
         [x2, y2, theta2],
                ...      ]

    - Conversion is done in place, so this function does not return anything.
    - Provide the MapMetaData object from a map message to specify the change in coordinates.
    - Same computation as ``map_to_world_slow``, but vectorized and inlined.
    """
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(poses[:, 0])
    poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
    poses[:, 1] = s * temp + c * poses[:, 1]

    # scale
    poses[:, :2] *= float(scale)

    # translate
    poses[:, 0] += map_info.origin.position.x
    poses[:, 1] += map_info.origin.position.y
    poses[:, 2] += angle


def world_to_map(poses: np.ndarray, map_info: MapMetaData) -> None:
    """Convert poses from world coordinate space (meters) to map coordinate space (pixels).

    ``poses`` is a two dimensional numpy array::

        [[x0, y0, theta0],
         [x1, y1, theta1],
         [x2, y2, theta2],
                ...      ]

    - Conversion is done in place, so this function does not return anything.
    - Provide the MapMetaData object from a map message to specify the change in coordinates.
    - Same computation as ``world_to_map_slow``, but vectorized and inlined.
    - You may have to transpose the returned x and y coordinates to index a pixel array.
    """
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)

    # translation
    poses[:, 0] -= map_info.origin.position.x
    poses[:, 1] -= map_info.origin.position.y

    # scale
    poses[:, :2] *= 1.0 / float(scale)

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(poses[:, 0])
    poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
    poses[:, 1] = s * temp + c * poses[:, 1]
    poses[:, 2] += angle


def world_to_map_slow(
    x: float, y: float, t: float, map_info: MapMetaData
) -> tuple[float, float, float]:
    """Convert (x, y, t) from world coordinates (meters) into map coordinates (pixels).

    Provide the MapMetaData object from a map message to specify the change in
    coordinates. This is the logical but slow implementation: for a lot of coordinate
    conversions, use ``world_to_map`` instead.
    """
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)
    rot = rotation_matrix(-angle)
    trans = np.array([[map_info.origin.position.x], [map_info.origin.position.y]])

    world = np.array([[x], [y]])
    map_c = rot * ((world - trans) / float(scale))
    return map_c[0, 0], map_c[1, 0], t - angle
