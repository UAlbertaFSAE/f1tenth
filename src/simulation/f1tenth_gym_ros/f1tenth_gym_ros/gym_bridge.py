# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import pathlib

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from f1tenth_gym.envs import F110Env
from f1tenth_gym.envs.env_config import (
    EnvConfig,
    ControlConfig,
    SimulationConfig,
    ObservationConfig,
    ResetConfig,
    LoopCounterMode,
)
from f1tenth_gym.envs.action import LongitudinalActionType, SteerActionType
from f1tenth_gym.envs.dynamic_models import (
    DynamicModel,
    get_f1tenth_vehicle_parameters,
    get_fullscale_vehicle_parameters,
    get_f1fifth_vehicle_parameters,
)
from f1tenth_gym.envs.integrators import IntegratorType
from f1tenth_gym.envs.lidar import LiDARConfig
from f1tenth_gym.envs.observation import ObservationType
from f1tenth_gym.envs.reset import ResetStrategy
from f1tenth_gym.envs.track import Track, Raceline


def _resolve_yaml_path(base_path: pathlib.Path) -> pathlib.Path:
    if base_path.suffix in (".yaml", ".yml"):
        return base_path
    candidates = (
        base_path.with_suffix(".yaml"),
        base_path.with_suffix(".yml"),
        base_path.parent / f"{base_path.stem}_map.yaml",
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return base_path.with_suffix(".yaml")


def _resolve_map_yaml_path(map_path: str) -> pathlib.Path | None:
    path = pathlib.Path(map_path)
    if not path.is_absolute():
        share_dir = pathlib.Path(get_package_share_directory("f1tenth_gym_ros"))
        path = share_dir / map_path
    yaml_path = _resolve_yaml_path(path)
    return yaml_path if yaml_path.exists() else None


def _load_track_from_yaml(map_yaml_path: pathlib.Path, scale: float) -> tuple[Track, bool]:
    track_spec = Track.load_spec(track=map_yaml_path.stem, filespec=str(map_yaml_path))
    track_spec.resolution = track_spec.resolution * scale
    track_spec.origin = (
        track_spec.origin[0] * scale,
        track_spec.origin[1] * scale,
        track_spec.origin[2],
    )

    image_path = map_yaml_path.parent / track_spec.image
    flip_op = getattr(Image, "Transpose", Image).FLIP_TOP_BOTTOM
    image = Image.open(image_path).transpose(flip_op)
    occupancy_map = np.array(image).astype(np.float32)
    occupancy_map[occupancy_map <= 128] = 0.0
    occupancy_map[occupancy_map > 128] = 255.0

    centerline_path = map_yaml_path.parent / f"{map_yaml_path.stem}_centerline.csv"
    raceline_path = map_yaml_path.parent / f"{map_yaml_path.stem}_raceline.csv"
    centerline = None
    raceline = None
    if centerline_path.exists():
        centerline = Raceline.from_centerline_file(centerline_path, track_scale=scale)
    if raceline_path.exists():
        raceline = Raceline.from_raceline_file(raceline_path, track_scale=scale)

    if raceline is None:
        raceline = centerline
    if centerline is None:
        centerline = raceline

    track = Track(
        spec=track_spec,
        filepath=str(map_yaml_path.absolute()),
        ext=image_path.suffix,
        occupancy_map=occupancy_map,
        centerline=centerline,
        raceline=raceline,
    )
    has_reference_line = centerline is not None or raceline is not None
    return track, has_reference_line

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.declare_parameter('ego_namespace', 'ego_racecar')
        self.declare_parameter('ego_odom_topic', 'odom')
        self.declare_parameter('ego_opp_odom_topic', 'opp_odom')
        self.declare_parameter('ego_scan_topic', 'scan')
        self.declare_parameter('ego_drive_topic', 'drive')
        self.declare_parameter('opp_namespace', 'opp_racecar')
        self.declare_parameter('opp_odom_topic', 'odom')
        self.declare_parameter('opp_ego_odom_topic', 'opp_odom')
        self.declare_parameter('opp_scan_topic', 'opp_scan')
        self.declare_parameter('opp_drive_topic', 'opp_drive')
        self.declare_parameter('lidar_enabled', True)
        self.declare_parameter('lidar_base_link_to_lidar_tf', [0.275, 0.0, 0.0])
        self.declare_parameter('lidar_noise_std', 0.01)
        self.declare_parameter('scan_num_beams', 1080)
        self.declare_parameter('scan_range_min', 0.0)
        self.declare_parameter('scan_range_max', 30.0)
        self.declare_parameter('scan_angle_min', -135.0)
        self.declare_parameter('scan_angle_max', 135.0)
        self.declare_parameter('map_path', 'levine')
        self.declare_parameter('map_img_ext', '.png')
        self.declare_parameter('num_agent', 1)
        self.declare_parameter('sx', 0.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)
        self.declare_parameter('sx1', 2.0)
        self.declare_parameter('sy1', 0.5)
        self.declare_parameter('stheta1', 0.0)
        self.declare_parameter('kb_teleop', True)
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('vehicle_params', 'f1tenth')
        self.declare_parameter('async_mode', True)
        # Flag to know whether to publish the sim time or not
        # Has to be different than use_sim_time so we can still use real time to trigger timer callbacks
        self.declare_parameter('use_sim_time_bridge', False)

        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if num_agents < 1 or num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        elif type(num_agents) != int:
            raise ValueError('num_agents should be an int.')

        self.vehicle_params = None
        vehicle_params_key = self.get_parameter('vehicle_params').value
        if vehicle_params_key == 'f1tenth':
            self.vehicle_params = get_f1tenth_vehicle_parameters()
        elif vehicle_params_key == 'fullscale':
            self.vehicle_params = get_fullscale_vehicle_parameters()
        elif vehicle_params_key == 'f1fifth':
            self.vehicle_params = get_f1fifth_vehicle_parameters()
        else:
            raise ValueError('vehicle_params should be either f1tenth, fullscale, or f1fifth.')

        scale = self.get_parameter('scale').value
        map_path = self.get_parameter('map_path').value
        map_yaml_path = _resolve_map_yaml_path(map_path)

        if map_yaml_path is not None:
            self.get_logger().info('Loading map from path: %s' % map_yaml_path)
            try:
                loaded_map = Track.from_track_path(map_yaml_path, track_scale=scale)
                has_reference_line = (
                    loaded_map.centerline is not None or loaded_map.raceline is not None
                )
            except (ValueError, FileNotFoundError) as ex:
                if isinstance(ex, FileNotFoundError) or "centerline" in str(ex) or "raceline" in str(ex):
                    loaded_map, has_reference_line = _load_track_from_yaml(map_yaml_path, scale)
                else:
                    raise
        else:
            self.get_logger().info('Loading map by name: %s' % map_path)
            loaded_map = Track.from_track_name(map_path, track_scale=scale)
            has_reference_line = loaded_map.centerline is not None or loaded_map.raceline is not None

        if not has_reference_line:
            self.get_logger().warning(
                'Map has no centerline/raceline; disabling frenet frame and lap counting.'
            )
            
        lidar_enabled = self.get_parameter('lidar_enabled').value
        scan_num_beams = self.get_parameter('scan_num_beams').value
        if not isinstance(scan_num_beams, int):
            if isinstance(scan_num_beams, float) and scan_num_beams.is_integer():
                scan_num_beams = int(scan_num_beams)
            else:
                raise ValueError('scan_num_beams must be an integer.')
        lidar_noise_std = self.get_parameter('lidar_noise_std').value
        lidar_base_link_to_lidar_tf = self.get_parameter(
            'lidar_base_link_to_lidar_tf'
        ).value
        if len(lidar_base_link_to_lidar_tf) != 3:
            raise ValueError('lidar_base_link_to_lidar_tf must be [x, y, yaw].')
        lidar_base_link_to_lidar_tf = tuple(lidar_base_link_to_lidar_tf)
        scan_range_min = self.get_parameter('scan_range_min').value
        scan_range_max = self.get_parameter('scan_range_max').value
        scan_angle_min = self.get_parameter('scan_angle_min').value
        scan_angle_max = self.get_parameter('scan_angle_max').value
        lidar_cfg = LiDARConfig(
            enabled=lidar_enabled,
            num_beams=scan_num_beams,
            range_min=scan_range_min,
            range_max=scan_range_max,
            angle_min=np.deg2rad(scan_angle_min),
            angle_max=np.deg2rad(scan_angle_max),
            noise_std=lidar_noise_std,
            base_link_to_lidar_tf=lidar_base_link_to_lidar_tf,
        )
        self.lidar_cfg = lidar_cfg

        loop_counter = (
            LoopCounterMode.FRENET_BASED if has_reference_line else LoopCounterMode.TOGGLE
        )
        compute_frenet = has_reference_line
        simulation_cfg = SimulationConfig(
            timestep=0.01,
            integrator_timestep=0.01,
            integrator=IntegratorType.RK4,
            dynamics_model=DynamicModel.ST,
            loop_counter=loop_counter,
            compute_frenet_frame=compute_frenet,
        )
        control_cfg = ControlConfig(
            longitudinal_mode=LongitudinalActionType.SPEED,
            steering_mode=SteerActionType.STEERING_ANGLE,
        )
        observation_cfg = ObservationConfig(type=ObservationType.DIRECT)
        reset_cfg = ResetConfig(strategy=ResetStrategy.MAP_RANDOM_STATIC)

        env_config = EnvConfig(
            map_name=loaded_map,
            map_scale=scale,
            params=self.vehicle_params,
            num_agents=num_agents,
            control_config=control_cfg,
            simulation_config=simulation_cfg,
            observation_config=observation_cfg,
            reset_config=reset_cfg,
            lidar_config=lidar_cfg,
            render_enabled=False,
        )
        self.env = F110Env(config=env_config, render_mode=None)

        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        self.angle_min = self.lidar_cfg.angle_min
        self.angle_max = self.lidar_cfg.angle_max
        self.angle_inc = self.lidar_cfg.angle_increment
        self.scan_range_min = self.lidar_cfg.range_min
        self.scan_range_max = self.lidar_cfg.range_max
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        self.scan_tf = self.lidar_cfg.base_link_to_lidar_tf
        
        if num_agents == 2:
            self.has_opp = True
            self.opp_namespace = self.get_parameter('opp_namespace').value
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_collision = False
            self.env.reset(options={"poses": np.array([[sx, sy, stheta], [sx1, sy1, stheta1]])})
            self._update_sim_state()

            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
            opp_drive_topic = self.get_parameter('opp_drive_topic').value

            ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
            opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value
        else:
            self.has_opp = False
            self.env.reset(options={"poses": np.array([[sx, sy, stheta]])})
            self._update_sim_state()

        if not self.get_parameter('async_mode').value:
            self.get_logger().info('Running in synchronous mode. Simulation will step only on new /drive messages.')
            # topic publishing timer slowly, fallback for if the controller is waiting for a first odom and scan
            self.timer = self.create_timer(1, self.timer_callback)
        else:
            self.get_logger().info('Running in asynchronous mode. Simulation will step using a timer callback.')
            # sim physical step timer
            self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
            # topic publishing timer
            self.timer = self.create_timer(0.004, self.timer_callback)

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False
        if num_agents == 2:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
            self.opp_drive_published = False
            
        if self.get_parameter('use_sim_time_bridge').value:
            self.get_logger().info('Using simulation time. Will publish /clock topic. Drive and odom will be as fast as possible.')
            self.clock_pub = self.create_publisher(Clock, '/clock', 10)
            if self.get_parameter('async_mode').value:
                # Set drive timer to 0 to trigger the callback asap
                self.drive_timer.timer_period_ns = 0
                self.timer.timer_period_ns = 0

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)
        if num_agents == 2:
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                opp_drive_topic,
                self.opp_drive_callback,
                10)
            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.opp_reset_callback,
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)

        self.sim_paused = False            
        self.pause_subscriber = self.create_subscription(
            Bool,
            '/pause_sim',
            self.pause_callback,
            10)

    def pause_callback(self, msg):
        self.sim_paused = msg.data
        self.get_logger().info(f"Simulation {'paused' if self.sim_paused else 'resumed'}")

    def drive_callback(self, drive_msg):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = np.clip(drive_msg.drive.steering_angle, self.vehicle_params.s_min, self.vehicle_params.s_max)
        
        if not self.get_parameter('async_mode').value:
            # step the sim immediately and publish odom and scan
            self.drive_timer_callback()
            self.timer_callback()

    def opp_drive_callback(self, drive_msg):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        self.opp_requested_speed = drive_msg.drive.speed
        self.opp_steer = np.clip(drive_msg.drive.steering_angle, self.vehicle_params.s_min, self.vehicle_params.s_max)
        
        if not self.get_parameter('async_mode').value:
            # step the sim immediately and publish odom and scan
            self.drive_timer_callback()
            self.timer_callback()

    def ego_reset_callback(self, pose_msg):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        rtheta = Rotation.from_quat([rqx, rqy, rqz, rqw]).as_euler('xyz')[2]
        if self.has_opp:
            opp_pose = [self.opp_pose[0], self.opp_pose[1], self.opp_pose[2]]
            self.env.reset(options={"poses": np.array([[rx, ry, rtheta], opp_pose])})
        else:
            self.env.reset(options={"poses": np.array([[rx, ry, rtheta]])})
        self._update_sim_state()

    def opp_reset_callback(self, pose_msg):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        if self.has_opp:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rqx = pose_msg.pose.orientation.x
            rqy = pose_msg.pose.orientation.y
            rqz = pose_msg.pose.orientation.z
            rqw = pose_msg.pose.orientation.w
            rtheta = Rotation.from_quat([rqx, rqy, rqz, rqw]).as_euler('xyz')[2]
            self.env.reset(options={"poses": np.array([self.ego_pose, [rx, ry, rtheta]])})
            self._update_sim_state()

    def teleop_callback(self, twist_msg):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self): 
        if self.sim_paused:
            return  # Skip stepping the sim if paused
        
        if not self.has_opp:
            _, _, self.done, _, _ = self.env.step(
                np.array([[self.ego_steer, self.ego_requested_speed]])
            )
        else:
            _, _, self.done, _, _ = self.env.step(
                np.array(
                    [
                        [self.ego_steer, self.ego_requested_speed],
                        [self.opp_steer, self.opp_requested_speed],
                    ]
                )
            )
        self._update_sim_state()
        if self.get_parameter('use_sim_time_bridge').value:
            clock_msg = Clock()
            sim_time = self.env.unwrapped.sim_time
            clock_msg.clock.sec = int(sim_time // 1.0)
            clock_msg.clock.nanosec = int((sim_time % 1.0) * 1e9)
            self.clock_pub.publish(clock_msg)

    def timer_callback(self):
        if self.sim_paused:
            return  # Skip stepping the sim if paused
        
        ts = self.get_clock().now().to_msg()
        if self.get_parameter('use_sim_time_bridge').value:
            # Ensure sim-time stamps the messages   
            sim_time = self.env.unwrapped.sim_time
            ts.sec = int(sim_time // 1.0)
            ts.nanosec = int((sim_time % 1.0) * 1e9)

        # pub scans
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        # convert each element to float from numpy.float32
        self.ego_scan = [float(x) for x in self.ego_scan]
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = self.scan_range_min
            opp_scan.range_max = self.scan_range_max
            self.opp_scan = [float(x) for x in self.opp_scan]
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)

        # pub tf
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        sim_state = self.env.unwrapped.sim.state
        scans = sim_state.scans
        poses = sim_state.poses
        std_state = sim_state.standard_state

        self.ego_scan = list(scans[0])
        self.ego_pose[0] = float(poses[0, 0])
        self.ego_pose[1] = float(poses[0, 1])
        self.ego_pose[2] = float(poses[0, 2])
        ego_speed = float(std_state[0, 3])
        ego_beta = float(std_state[0, 6])
        self.ego_speed[0] = float(ego_speed * np.cos(ego_beta))
        self.ego_speed[1] = float(ego_speed * np.sin(ego_beta))
        self.ego_speed[2] = float(std_state[0, 5])

        if self.has_opp:
            self.opp_scan = list(scans[1])
            self.opp_pose[0] = float(poses[1, 0])
            self.opp_pose[1] = float(poses[1, 1])
            self.opp_pose[2] = float(poses[1, 2])
            opp_speed = float(std_state[1, 3])
            opp_beta = float(std_state[1, 6])
            self.opp_speed[0] = float(opp_speed * np.cos(opp_beta))
            self.opp_speed[1] = float(opp_speed * np.sin(opp_beta))
            self.opp_speed[2] = float(std_state[1, 5])

        

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = Rotation.from_euler('xyz', [0., 0., self.ego_pose[2]]).as_quat()
        ego_odom.pose.pose.orientation.x = ego_quat[0]
        ego_odom.pose.pose.orientation.y = ego_quat[1]
        ego_odom.pose.pose.orientation.z = ego_quat[2]
        ego_odom.pose.pose.orientation.w = ego_quat[3]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = Rotation.from_euler('xyz', [0., 0., self.opp_pose[2]]).as_quat()
            opp_odom.pose.pose.orientation.x = opp_quat[0]
            opp_odom.pose.pose.orientation.y = opp_quat[1]
            opp_odom.pose.pose.orientation.z = opp_quat[2]
            opp_odom.pose.pose.orientation.w = opp_quat[3]
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = Rotation.from_euler('xyz', [0.0, 0.0, self.ego_pose[2]]).as_quat()
        ego_t.rotation.x = ego_quat[0]
        ego_t.rotation.y = ego_quat[1]
        ego_t.rotation.z = ego_quat[2]
        ego_t.rotation.w = ego_quat[3]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = 'map'
        ego_ts.child_frame_id = self.ego_namespace + '/base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x = self.opp_pose[0]
            opp_t.translation.y = self.opp_pose[1]
            opp_t.translation.z = 0.0
            opp_quat = Rotation.from_euler('xyz', [0.0, 0.0, self.opp_pose[2]]).as_quat()
            opp_t.rotation.x = opp_quat[0]
            opp_t.rotation.y = opp_quat[1]
            opp_t.rotation.z = opp_quat[2]
            opp_t.rotation.w = opp_quat[3]

            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp = ts
            opp_ts.header.frame_id = 'map'
            opp_ts.child_frame_id = self.opp_namespace + '/base_link'
            self.br.sendTransform(opp_ts)

    def _publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = Rotation.from_euler('xyz', [0., 0., self.ego_steer]).as_quat()
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[0]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[3]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = Rotation.from_euler('xyz', [0., 0., self.opp_steer]).as_quat()
            opp_wheel_ts.transform.rotation.x = opp_wheel_quat[0]
            opp_wheel_ts.transform.rotation.y = opp_wheel_quat[1]
            opp_wheel_ts.transform.rotation.z = opp_wheel_quat[2]
            opp_wheel_ts.transform.rotation.w = opp_wheel_quat[3]
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)

    def _publish_laser_transforms(self, ts):
        scan_quat = Rotation.from_euler('xyz', [0.0, 0.0, self.scan_tf[2]]).as_quat()
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_tf[0]
        ego_scan_ts.transform.translation.y = self.scan_tf[1]
        ego_scan_ts.transform.translation.z = 0.0
        ego_scan_ts.transform.rotation.x = scan_quat[0]
        ego_scan_ts.transform.rotation.y = scan_quat[1]
        ego_scan_ts.transform.rotation.z = scan_quat[2]
        ego_scan_ts.transform.rotation.w = scan_quat[3]
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_tf[0]
            opp_scan_ts.transform.translation.y = self.scan_tf[1]
            opp_scan_ts.transform.translation.z = 0.0
            opp_scan_ts.transform.rotation.x = scan_quat[0]
            opp_scan_ts.transform.rotation.y = scan_quat[1]
            opp_scan_ts.transform.rotation.z = scan_quat[2]
            opp_scan_ts.transform.rotation.w = scan_quat[3]
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
