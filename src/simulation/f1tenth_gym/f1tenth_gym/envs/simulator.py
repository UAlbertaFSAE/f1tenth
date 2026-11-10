"""Core F110 simulator handling multi-agent dynamics, LiDAR, and collision logic."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Optional

import numpy as np

from .action import (
    LongitudinalActionType,
    SteerActionType,
    longitudinal_action_from_type,
    steer_action_from_type,
)
from .collision_models import collision_multiple, get_vertices
from .dynamic_models import DynamicModel, VehicleParameters
from .env_config import EnvConfig
from .lidar import ScanSimulator2D, check_ttc_jit, ray_cast
from .state import SimulationState
from .track import Track

DynamicsFn = Callable[[np.ndarray, np.ndarray, np.ndarray], np.ndarray]
IntegratorFn = Callable[[DynamicsFn, np.ndarray, np.ndarray, float, np.ndarray], np.ndarray]
AccelerationFn = Callable[[float, np.ndarray, VehicleParameters], float]
SteeringFn = Callable[[float, np.ndarray, VehicleParameters], float]

@dataclass
class ScanCache:
    """Precomputed LiDAR geometry for collision detection.

    Attributes:
        angles: Beam angles relative to vehicle heading.
        cosines: Cosines of beam angles.
        side_distances: Distance from vehicle center to edge per beam.
    """

    angles: np.ndarray
    cosines: np.ndarray
    side_distances: np.ndarray


class F110Simulator:
    """Core simulator for F1TENTH multi-agent racing.

    Handles vehicle dynamics integration, LiDAR simulation, and collision detection
    for all agents in a single state-driven update loop.

    Attributes:
        state: Current simulation state containing poses, velocities, scans.
        track: The racing track being simulated.
        vehicle_params: Physical parameters of the vehicle.
        num_agents: Number of agents in the simulation.
    """

    ttc_threshold: float = 0.005

    def __init__(
        self,
        *,
        env_config: EnvConfig,
        vehicle_params: VehicleParameters,
        model: DynamicModel,
        dynamics_fn: DynamicsFn,
        integrator_fn: IntegratorFn,
        longitudinal_type: LongitudinalActionType,
        steering_type: SteerActionType,
        track: Optional[Track],
        seed: int,
    ) -> None:
        self.config = env_config
        self.vehicle_params = vehicle_params
        self.model = model
        self.dynamics_fn = dynamics_fn
        self.integrator_fn = integrator_fn
        self.track = track
        self.seed = seed

        self.num_agents = env_config.num_agents
        self.ego_idx = env_config.ego_index
        self.time_step = env_config.simulation_config.timestep
        self.integrator_dt = env_config.simulation_config.integrator_timestep
        if not np.isclose(self.time_step % self.integrator_dt, 0.0):
            raise ValueError("time_step must be an integer multiple of integrator_timestep")
        self.substeps = max(1, int(round(self.time_step / self.integrator_dt)))

        self.longitudinal_fn: AccelerationFn = longitudinal_action_from_type(longitudinal_type)
        self.steering_fn: SteeringFn = steer_action_from_type(steering_type)

        self.longitudinal_type = longitudinal_type
        self.steering_type = steering_type

        self.params_array = self.vehicle_params.to_array(self.model)

        # Allocate simulation state buffers
        initial_state = self.model.get_initial_state(params=self.params_array)
        self.state_dim = initial_state.shape[0]
        self.control_dim = self.model.control_dim
        scan_size = env_config.lidar_config.num_beams if env_config.lidar_config.enabled else 1
        self.state = SimulationState.allocate(
            num_agents=self.num_agents,
            state_dim=self.state_dim,
            scan_size=scan_size,
            control_dim=self.control_dim,
            delay_steps=env_config.control_config.steer_delay_steps,
        )

        # Static helpers
        self.standard_state_fn = self.model.get_standardized_state_fn()
        self.scan_enabled = env_config.lidar_config.enabled
        self.scan_max_range = env_config.lidar_config.maximum_range

        self.scan_sims: list[ScanSimulator2D] = []
        self.scan_rngs: list[np.random.Generator] = []
        self.scan_cache: list[ScanCache] = []
        if self.scan_enabled:
            lidar_cfg = env_config.lidar_config
            for agent_index in range(self.num_agents):
                rng = np.random.default_rng(seed + agent_index)
                simulator = ScanSimulator2D(
                    lidar_cfg.num_beams,
                    lidar_cfg.field_of_view,
                    angle_min=lidar_cfg.angle_min,
                    angle_max=lidar_cfg.angle_max,
                    std_dev=lidar_cfg.noise_std,
                    min_range=lidar_cfg.range_min,
                    max_range=lidar_cfg.range_max,
                )
                if self.track is not None:
                    simulator.set_map(self.track, env_config.map_scale)
                cache = self._build_scan_cache(simulator, self.vehicle_params)
                self.scan_sims.append(simulator)
                self.scan_rngs.append(rng)
                self.scan_cache.append(cache)

        # Geometry buffers for collision checks
        self.agent_vertices = np.zeros((self.num_agents, 4, 2), dtype=np.float64)

    # ---------------------------------------------------------------------
    # Public API
    # ---------------------------------------------------------------------
    def set_map(self, track: Track, map_scale: float = 1.0) -> None:
        """Set or update the track used for simulation.

        Args:
            track: Track object with occupancy map and reference lines.
            map_scale: Scale factor applied to the map.
        """
        self.track = track
        if not self.scan_enabled:
            return
        for simulator in self.scan_sims:
            simulator.set_map(track, map_scale)

    def update_params(self, vehicle_params: VehicleParameters, agent_idx: int = -1) -> None:
        """Update vehicle parameters for all agents.

        Args:
            vehicle_params: New vehicle physical parameters.
            agent_idx: Agent index (-1 for all agents, per-agent not supported).
        """
        if agent_idx >= 0:
            raise NotImplementedError("Per-agent parameter updates are not supported")
        self.vehicle_params = vehicle_params
        self.params_array = vehicle_params.to_array(self.model)
        if self.scan_enabled:
            for i, simulator in enumerate(self.scan_sims):
                self.scan_cache[i] = self._build_scan_cache(simulator, vehicle_params)

    def reset(self, poses: np.ndarray, *, option: str = "pose") -> None:
        """Reset all agents to initial positions.

        Args:
            poses: Initial positions, shape (num_agents, 3) for poses or
                   (num_agents, state_dim) for full state.
            option: Reset mode - "pose" for (x, y, theta) or "state" for full state.
        """
        if poses.shape[0] != self.num_agents:
            raise ValueError("Number of poses does not match number of agents")

        self.state.reset()
        if self.scan_enabled:
            for idx in range(self.num_agents):
                self.scan_rngs[idx] = np.random.default_rng(self.seed + idx)
        for i in range(self.num_agents):
            if option == "pose":
                self.state.state[i] = self.model.get_initial_state(
                    pose=poses[i], params=self.params_array
                ).astype(np.float32)
            elif option == "state":
                if poses.shape[1] != self.state_dim:
                    raise ValueError("State reset has incorrect dimension")
                self.state.state[i] = poses[i].astype(np.float32)
            else:
                raise ValueError("Unsupported reset option")
            self.state.standard_state[i] = self.standard_state_fn(self.state.state[i]).astype(
                np.float32
            )
            self.state.poses[i] = np.array(
                [self.state.state[i, 0], self.state.state[i, 1], self.state.state[i, 4]],
                dtype=np.float32,
            )
            if self.config.simulation_config.compute_frenet_frame and self.track is not None:
                self.state.frenet[i] = np.array(
                    self.track.cartesian_to_frenet(
                        float(self.state.state[i, 0]),
                        float(self.state.state[i, 1]),
                        float(self.state.state[i, 4]),
                        use_s_guess=False,
                    ),
                    dtype=np.float32,
                )
    def step(self, control_inputs: np.ndarray) -> None:
        """Advance simulation by one timestep.

        Args:
            control_inputs: Control commands, shape (num_agents, 2) with
                            [steering, acceleration/speed] per agent.
        """
        if control_inputs.shape != (self.num_agents, self.control_dim):
            raise ValueError("Control input has incorrect shape")

        steer_commands = control_inputs[:, 0].astype(np.float32)
        accel_commands = control_inputs[:, 1].astype(np.float32)
        self.state.control_input[:, 1] = accel_commands

        if self.state.delay_buffer is not None:
            delayed_steer = self.state.push_delay(steer_commands)
        else:
            delayed_steer = steer_commands
        self.state.control_input[:, 0] = delayed_steer

        for agent_idx in range(self.num_agents):
            state = self.state.state[agent_idx]
            steer_effort = self.steering_fn(delayed_steer[agent_idx], state, self.vehicle_params)
            accel_effort = self.longitudinal_fn(accel_commands[agent_idx], state, self.vehicle_params)
            control_vector = np.array([steer_effort, accel_effort], dtype=np.float32)

            for _ in range(self.substeps):
                state = self.integrator_fn(
                    self.dynamics_fn,
                    state,
                    control_vector,
                    self.integrator_dt,
                    self.params_array,
                )
            state[4] = (state[4] + np.pi) % (2 * np.pi) - np.pi
            self.state.state[agent_idx] = state.astype(np.float32)
            self.state.standard_state[agent_idx] = self.standard_state_fn(state).astype(
                np.float32
            )
            self.state.poses[agent_idx] = np.array(
                [state[0], state[1], state[4]], dtype=np.float32
            )

        if self.config.simulation_config.compute_frenet_frame and self.track is not None:
            for agent_idx in range(self.num_agents):
                pose = self.state.poses[agent_idx]
                self.state.frenet[agent_idx] = np.array(
                    self.track.cartesian_to_frenet(
                        float(pose[0]), float(pose[1]), float(pose[2])
                    ),
                    dtype=np.float32,
                )

        if self.scan_enabled:
            self._update_scans()
        else:
            self.state.scans.fill(0.0)
            self.state.collisions.fill(0.0)

        self._update_agent_collisions()
        self.state.sim_time += self.time_step

    # ------------------------------------------------------------------
    # Convenience accessors
    # ------------------------------------------------------------------
    @property
    def agent_scans(self) -> np.ndarray:
        return self.state.scans

    @property
    def collisions(self) -> np.ndarray:
        return self.state.collisions

    @property
    def scan_num_beams(self) -> int:
        return self.config.lidar_config.num_beams if self.scan_enabled else 0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_scan_cache(self, simulator: ScanSimulator2D, vehicle_params: VehicleParameters) -> ScanCache:
        """Build precomputed scan geometry for collision detection.

        Computes side_distances as the distance from the LiDAR position to the
        vehicle body edge for each beam angle. This correctly accounts for the
        LiDAR offset (base_link_to_lidar_tf) and collision body center offset
        (collision_body_center_x/y) from the vehicle parameters, with an
        adjustment for collision checks when the pose is at the center of gravity.
        """
        num_beams = simulator.num_beams
        angles = np.zeros(num_beams, dtype=np.float32)
        cosines = np.zeros(num_beams, dtype=np.float32)
        side_distances = np.zeros(num_beams, dtype=np.float32)

        half_length = float(vehicle_params.length) / 2.0
        half_width = float(vehicle_params.width) / 2.0
        if not np.isfinite(half_length) or not np.isfinite(half_width):
            raise ValueError("Vehicle length and width must be finite to build LiDAR cache")

        # Get LiDAR offset from base_link
        lidar_tf = self.config.lidar_config.base_link_to_lidar_tf
        lidar_dx, lidar_dy, lidar_dtheta = lidar_tf

        # Get collision body center offset from base_link
        body_dx = vehicle_params.collision_body_center_x
        body_dy = vehicle_params.collision_body_center_y

        # If state is referenced at the center of gravity, base_link is behind it by lr.
        # Leave LiDAR in the state frame to avoid shifting the scan origin.
        base_dx = 0.0
        if self.model != DynamicModel.KS:
            base_dx = -float(vehicle_params.lr)
            if not math.isfinite(base_dx):
                base_dx = 0.0

        # Compute LiDAR position relative to collision body center
        # (the collision body is centered at (body_dx, body_dy) from base_link)
        body_x = base_dx + body_dx
        lidar_x_in_body = lidar_dx - body_x
        lidar_y_in_body = lidar_dy - body_dy

        increment = simulator.get_increment()
        angle_min = simulator.angle_min

        for idx in range(num_beams):
            # Beam angle relative to vehicle heading
            beam_angle = angle_min + idx * increment
            angles[idx] = beam_angle
            cosines[idx] = math.cos(beam_angle)

            # Ray angle accounts for LiDAR yaw offset
            ray_angle = beam_angle + lidar_dtheta
            dir_cos = math.cos(ray_angle)
            dir_sin = math.sin(ray_angle)

            # Compute distance from LiDAR to collision body edge
            side_distances[idx] = self._ray_to_rect_distance(
                lidar_x_in_body, lidar_y_in_body,
                dir_cos, dir_sin,
                half_length, half_width,
            )

        return ScanCache(angles=angles, cosines=cosines, side_distances=side_distances)


    def _lidar_pose_from_base(self, pose: np.ndarray) -> np.ndarray:
        tf = self.config.lidar_config.base_link_to_lidar_tf
        dx, dy, dtheta = tf
        if dx == 0.0 and dy == 0.0 and dtheta == 0.0:
            return pose
        cos_yaw = math.cos(pose[2])
        sin_yaw = math.sin(pose[2])
        scan_x = pose[0] + dx * cos_yaw - dy * sin_yaw
        scan_y = pose[1] + dx * sin_yaw + dy * cos_yaw
        scan_theta = pose[2] + dtheta
        return np.array([scan_x, scan_y, scan_theta], dtype=pose.dtype)

    def _collision_pose_from_base(self, pose: np.ndarray) -> np.ndarray:
        """Transform pose to collision body center.

        Args:
            pose: Pose of the model state (base_link for KS, CoG for others).

        Returns:
            Collision body center pose in world frame.
        """
        base_dx = 0.0
        base_dy = 0.0
        if self.model != DynamicModel.KS:
            base_dx = -float(self.vehicle_params.lr)
            if not math.isfinite(base_dx):
                base_dx = 0.0
        dx = base_dx + float(self.vehicle_params.collision_body_center_x)
        dy = base_dy + float(self.vehicle_params.collision_body_center_y)
        if dx == 0.0 and dy == 0.0:
            return pose
        cos_yaw = math.cos(pose[2])
        sin_yaw = math.sin(pose[2])
        body_x = pose[0] + dx * cos_yaw - dy * sin_yaw
        body_y = pose[1] + dx * sin_yaw + dy * cos_yaw
        return np.array([body_x, body_y, pose[2]], dtype=pose.dtype)

    @staticmethod
    def _ray_to_rect_distance(
        origin_x: float,
        origin_y: float,
        dir_cos: float,
        dir_sin: float,
        half_length: float,
        half_width: float,
    ) -> float:
        """Compute distance from a point to the rectangle boundary along a ray.

        Args:
            origin_x, origin_y: Ray origin point (e.g., LiDAR position in body frame).
            dir_cos, dir_sin: Ray direction as (cos(angle), sin(angle)).
            half_length: Half of vehicle length (x extent from center).
            half_width: Half of vehicle width (y extent from center).

        Returns:
            Distance from origin to rectangle edge along the ray direction.
            Returns 0.0 if origin is outside the rectangle.
        """
        x_min, x_max = -half_length, half_length
        y_min, y_max = -half_width, half_width

        eps = 1e-9
        if not (x_min - eps <= origin_x <= x_max + eps and
                y_min - eps <= origin_y <= y_max + eps):
            return 0.0

        min_t = float('inf')

        if abs(dir_cos) > eps:
            t = (x_max - origin_x) / dir_cos
            if t > eps:
                y_intersect = origin_y + t * dir_sin
                if y_min - eps <= y_intersect <= y_max + eps:
                    min_t = min(min_t, t)

            t = (x_min - origin_x) / dir_cos
            if t > eps:
                y_intersect = origin_y + t * dir_sin
                if y_min - eps <= y_intersect <= y_max + eps:
                    min_t = min(min_t, t)

        if abs(dir_sin) > eps:
            t = (y_max - origin_y) / dir_sin
            if t > eps:
                x_intersect = origin_x + t * dir_cos
                if x_min - eps <= x_intersect <= x_max + eps:
                    min_t = min(min_t, t)

            t = (y_min - origin_y) / dir_sin
            if t > eps:
                x_intersect = origin_x + t * dir_cos
                if x_min - eps <= x_intersect <= x_max + eps:
                    min_t = min(min_t, t)

        if min_t == float('inf'):
            return 0.0

        return float(min_t)


    def _update_scans(self) -> None:
        for agent_idx, simulator in enumerate(self.scan_sims):
            pose = self.state.poses[agent_idx]
            scan_pose = self._lidar_pose_from_base(pose)

            # Get noise-free scan for collision detection
            scan_clean = simulator.scan(scan_pose, rng=None)
            cache = self.scan_cache[agent_idx]

            # Collision check uses noise-free scan
            in_collision = check_ttc_jit(
                scan_clean,
                self.state.standard_state[agent_idx, 3],
                cache.angles,
                cache.cosines,
                cache.side_distances,
                self.ttc_threshold,
            )
            if in_collision:
                self.state.state[agent_idx, 3:] = 0.0
                self.state.collisions[agent_idx] = 1.0
            else:
                self.state.collisions[agent_idx] = 0.0

            # Ray cast against other agents (also noise-free)
            origin = scan_pose.astype(np.float64)
            adjusted_scan = scan_clean
            for opp_idx in range(self.num_agents):
                if opp_idx == agent_idx:
                    continue
                opp_pose = self.state.poses[opp_idx]
                opp_collision_pose = self._collision_pose_from_base(opp_pose)
                opp_vertices = get_vertices(
                    np.array([opp_collision_pose[0], opp_collision_pose[1], opp_collision_pose[2]], dtype=np.float64),
                    self.vehicle_params.length,
                    self.vehicle_params.width,
                )
                adjusted_scan = ray_cast(origin, adjusted_scan, cache.angles, opp_vertices)

            # Add noise for observation output only
            noisy_scan = adjusted_scan + self.scan_rngs[agent_idx].normal(
                0.0, simulator.std_dev, size=simulator.num_beams
            )
            self.state.scans[agent_idx] = np.clip(
                noisy_scan, simulator.min_range, simulator.max_range
            ).astype(np.float32)

    def _update_agent_collisions(self) -> None:
        for agent_idx in range(self.num_agents):
            pose = self.state.poses[agent_idx]
            collision_pose = self._collision_pose_from_base(pose)
            self.agent_vertices[agent_idx] = get_vertices(
                np.array([collision_pose[0], collision_pose[1], collision_pose[2]], dtype=np.float64),
                self.vehicle_params.length,
                self.vehicle_params.width,
            )
        collisions, _ = collision_multiple(self.agent_vertices)
        self.state.collisions = np.maximum(self.state.collisions, collisions.astype(np.float32))
