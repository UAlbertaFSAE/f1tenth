from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np

from .dynamic_models import (
    VehicleParameters,
)
from .simulator import F110Simulator
from .env_config import (
    EnvConfig,
    LoopCounterMode,
)
from .integrators import integrator_from_type
from .action import (
    get_action_space,
    from_single_to_multi_action_space,
)
from .observation import ObservationType, observation_factory
from .reset import make_reset_fn
from .rendering import make_renderer
from .track import Track


class F110Env(gym.Env):
    """
    OpenAI Gym environment for F1TENTH autonomous racing.

    Simulates 1/10th scale autonomous race cars with realistic physics,
    LiDAR sensing, and collision detection.

    Attributes:
        track: The racing track used for simulation.
        sim: The underlying F110Simulator instance.
        num_agents: Number of agents in the environment.
        ego_idx: Index of the ego agent.
        vehicle_params: Vehicle parameters for dynamics.
    """

    metadata = {"render_modes": ["human", "human_fast", "rgb_array", "unlimited"], "render_fps": 100}

    def __init__(
        self,
        config: EnvConfig = EnvConfig(),
        render_mode=None,
    ):
        super().__init__()
        if isinstance(config, EnvConfig):
            resolved_config = config
        else:
            raise TypeError("config must be an EnvConfig instance")

        self.env_config = resolved_config
        self.render_mode = render_mode
        self.renderer = None
        self.render_spec = None
        self.render_obs = None

        self._apply_env_config()
        self._initialize_components()

    def configure(self, config: EnvConfig | None) -> None:
        """
        Reconfigure the environment with new settings.

        Args:
            config: New environment configuration, or None to skip.
        """
        if config is None:
            return
        if not isinstance(config, EnvConfig):
            raise TypeError("config must be an EnvConfig or None")

        self.env_config = config
        self._apply_env_config()
        self._initialize_components()

    def _apply_env_config(self) -> None:
        cfg = self.env_config

        self.seed = cfg.seed
        self.map = cfg.map_name
        self.map_scale = cfg.map_scale

        self.vehicle_params = cfg.params

        self.num_agents = cfg.num_agents
        self.ego_idx = cfg.ego_index

        self.control_cfg = cfg.control_config
        self.simulation_cfg = cfg.simulation_config
        self.observation_cfg = cfg.observation_config
        self.reset_cfg = cfg.reset_config
        self.lidar_cfg = cfg.lidar_config

        self.longitudinal_action_type = self.control_cfg.longitudinal_mode
        self.steer_action_type = self.control_cfg.steering_mode
        self.steer_delay_steps = self.control_cfg.steer_delay_steps

        self.timestep = self.simulation_cfg.timestep
        self.integrator_fn = integrator_from_type(self.simulation_cfg.integrator)
        self.model = self.simulation_cfg.dynamics_model
        self.loop_counter_mode = self.simulation_cfg.loop_counter
        self.compute_frenet = self.simulation_cfg.compute_frenet_frame
        self.max_laps = self.simulation_cfg.max_laps

        self.collision_check_mode = cfg.collision_check
        self.render_enabled = cfg.render_enabled

    def _resolve_track(self) -> Track:
        map_source = self.map
        if isinstance(map_source, Track):
            return map_source
        if isinstance(map_source, (str, Path)):
            map_str = str(map_source)
            map_path = Path(map_source)
            if "/" in map_str or "\\" in map_str or map_path.suffix:
                return Track.from_track_path(map_path, track_scale=self.map_scale)
            return Track.from_track_name(map_str, track_scale=self.map_scale)
        raise TypeError("map must be a Track instance or a path/name string")

    def _initialize_components(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
        self.renderer = None
        self.render_spec = None

        self.track = self._resolve_track()

        self.sim = F110Simulator(
            env_config=self.env_config,
            vehicle_params=self.vehicle_params,
            model=self.model,
            dynamics_fn=self.model.f_dynamics,
            integrator_fn=self.integrator_fn,
            longitudinal_type=self.longitudinal_action_type,
            steering_type=self.steer_action_type,
            track=self.track,
            seed=self.seed,
        )
        if isinstance(self.track, Track):
            self.sim.set_map(self.track, self.map_scale)

        self.agent_ids = [f"agent_{i}" for i in range(self.num_agents)]

        self.agents_prev_s = [None] * self.num_agents
        self.lap_times = np.zeros((self.num_agents,))
        self.lap_times_finish = np.zeros((self.num_agents,))
        self.lap_counts = np.zeros((self.num_agents,))
        self.sim_time = 0.0

        obs_kwargs: dict[str, Any] = {"type": self.observation_cfg.type}
        if self.observation_cfg.features is not None:
            obs_kwargs["features"] = self.observation_cfg.features
        self.observation_type = observation_factory(env=self, **obs_kwargs)
        self.observation_space = self.observation_type.space()
        self.render_obs_type = observation_factory(env=self, type=ObservationType.DIRECT)
        self.render_obs = None

        single_action_space = get_action_space(
            self.longitudinal_action_type,
            self.steer_action_type,
            self.vehicle_params,
        )
        self.action_space = from_single_to_multi_action_space(
            single_action_space, self.num_agents
        )

        self.reset_fn = make_reset_fn(
            track=self.track,
            num_agents=self.num_agents,
            type=self.reset_cfg.strategy,
        )

        base_fps = int(1.0 / self.timestep) if self.timestep > 0 else 0
        if self.render_mode == "human_fast":
            render_fps = base_fps * 10
        elif self.render_mode == "unlimited":
            render_fps = float("inf")
        else:
            render_fps = base_fps
        self.metadata["render_fps"] = render_fps

        if self.render_enabled:
            self.renderer, self.render_spec = make_renderer(
                params=self.vehicle_params,
                track=self.track,
                agent_ids=self.agent_ids,
                render_mode=self.render_mode,
                render_fps=self.metadata["render_fps"],
            )
        else:
            self.renderer = None
            self.render_spec = None

    def _check_done(self):
        """
        Check if the current rollout is done
        """
        if (
            self.loop_counter_mode is LoopCounterMode.FRENET_BASED
            and self.compute_frenet
            and self.track is not None
        ):
            s_frame_max = self.track.centerline.spline.s_frame_max
            for ind in range(self.num_agents):
                current_s = float(self.sim.state.frenet[ind, 0])
                if self.agents_prev_s[ind] is None:
                    self.agents_prev_s[ind] = current_s
                    continue
                if (
                    self.agents_prev_s[ind] - current_s > s_frame_max * 0.85
                    and self.sim_time > self.timestep
                ):
                    self.lap_counts[ind] += 1
                    self.lap_times[ind] = self.sim_time - self.lap_times_finish[ind]
                    self.lap_times_finish[ind] = self.sim_time
                self.agents_prev_s[ind] = current_s

        done = bool(self.sim.collisions[self.ego_idx])
        if self.max_laps is not None:
            done = done or (self.lap_counts[self.ego_idx] >= self.max_laps)
        return done

    def step(self, action):
        """
        Step function for the gym env

        Args:
            action (np.ndarray(num_agents, 2))

        Returns:
            obs (dict): observation of the current step
            reward (float, default=self.timestep): step reward, currently is physics timestep
            done (bool): if the simulation is done
            info (dict): auxillary information dictionary
        """

        # call simulation step
        self.sim.step(action)

        # check done
        done = self._check_done()

        # observation
        obs = self.observation_type.observe()
        if self.render_enabled:
            if self.observation_cfg.type is ObservationType.DIRECT:
                # for direct observation, also update the render_obs
                self.render_obs = copy.deepcopy(obs)
            else:
                # for other observation types, update the render_obs
                self.render_obs = copy.deepcopy(self.render_obs_type.observe())

        # times
        reward = self.timestep
        self.sim_time = self.sim.state.sim_time

        truncated = False
        info = {"lap_times": self.lap_times, "lap_counts": self.lap_counts, "sim_time": self.sim_time}

        return obs, reward, done, truncated, info

    def reset(self, seed=None, options=None):
        """
        Reset the gym environment by given poses

        Args:
            seed: random seed for the reset
            options: dictionary of options for the reset containing initial poses of the agents

        Returns:
            obs (dict): observation of the current step
            reward (float, default=self.timestep): step reward, currently is physics timestep
            done (bool): if the simulation is done
            info (dict): auxillary information dictionary
        """
        super().reset(seed=seed)

        self.sim_time = 0.0
        self.agents_prev_s = [None] * self.num_agents
        self.lap_counts.fill(0.0)
        self.lap_times.fill(0.0)
        self.lap_times_finish.fill(0.0)
        if options is not None and "poses" in options:
            poses = options["poses"]
            option = "pose"
        elif options is not None and "states" in options:
            poses = options["states"]
            option = "state"
        else:
            poses = self.reset_fn.sample(self.np_random)
            option = "pose"


        if option == "pose":
            assert isinstance(poses, np.ndarray) and poses.shape == (
                self.num_agents,
                3,
            ), "Initial poses must be a numpy array of shape (num_agents, 3)"
        elif option == "state":
            assert isinstance(poses, np.ndarray) and poses.shape == (
                self.num_agents,
                self.model.state_dim,
            ), f"Initial full state must be a numpy array of shape (num_agents, {self.model.state_dim})"
        else:
            raise ValueError("Invalid reset option.")

        self.sim.reset(poses, option=option)

        sim_poses = self.sim.state.poses
        self.start_xs = sim_poses[:, 0]
        self.start_ys = sim_poses[:, 1]
        self.start_thetas = sim_poses[:, 2]
        self.start_rot = np.array(
            [
                [
                    np.cos(-self.start_thetas[self.ego_idx]),
                    -np.sin(-self.start_thetas[self.ego_idx]),
                ],
                [
                    np.sin(-self.start_thetas[self.ego_idx]),
                    np.cos(-self.start_thetas[self.ego_idx]),
                ],
            ]
        )

        obs = self.observation_type.observe()
        if self.render_enabled:
            if self.observation_cfg.type is ObservationType.DIRECT:
                self.render_obs = copy.deepcopy(obs)
            else:
                self.render_obs = copy.deepcopy(self.render_obs_type.observe())

        info = {"lap_times": self.lap_times, "lap_counts": self.lap_counts, "sim_time": self.sim_time}

        return obs, info

    def update_map(self, map_name: Track | str):
        """
        Updates the map used by simulation

        Args:
            map_name (Track | str): name of the map, path to map, or Track instance

        Returns:
            None
        """
        new_config = self.env_config.with_updates(map_name=map_name)
        self.configure(new_config)

    def update_params(self, params, index=-1):
        """
        Update the shared vehicle parameters used by the simulator and renderers.

        Args:
            params (VehicleParameters): new vehicle parameters.
            index (int, default=-1): if >= 0 then only update a specific agent's params

        Returns:
            None
        """
        if index >= 0:
            raise NotImplementedError(
                "Per-agent parameter updates are not supported in the simplified simulator"
            )
        if isinstance(params, VehicleParameters):
            vehicle_params = params
        else:
            raise TypeError("params must be a VehicleParameters instance")

        self.vehicle_params = vehicle_params
        self.env_config = self.env_config.with_updates(params=vehicle_params)

        self.sim.update_params(self.vehicle_params)
        if hasattr(self, "renderer") and self.renderer is not None:
            self.renderer.update_params(self.vehicle_params)
        if hasattr(self, "action_space"):
            single_action_space = get_action_space(
                self.longitudinal_action_type,
                self.steer_action_type,
                self.vehicle_params,
            )
            self.action_space = from_single_to_multi_action_space(
                single_action_space, self.num_agents
            )            

    def add_render_callback(self, callback_func):
        """
        Add extra drawing function to call during rendering.

        Args:
            callback_func (function (EnvRenderer) -> None): custom function to called during render()
        """
        if self.render_enabled and self.renderer is not None:
            self.renderer.add_renderer_callback(callback_func)

    def render(self, mode="human"):
        """
        Renders the environment with pyglet. Use mouse scroll in the window to zoom in/out, use mouse click drag to pan.
        Shows the agents, the map, current fps (bottom left corner), and the race information near as text.

        Args:
            mode (str, default='human'): rendering mode, currently supports:
                'human': slowed down rendering such that the env is rendered in a way that sim time elapsed is close to real time elapsed
                'human_fast': render as fast as possible

        Returns:
            None
        """
        # NOTE: separate render (manage render-mode) from render_frame (actual rendering with pyglet)

        if self.render_mode not in self.metadata["render_modes"] or not self.render_enabled:
            return

        self.renderer.update(obs=self.render_obs)
        return self.renderer.render()

    def close(self):
        """
        Ensure renderer is closed upon deletion
        """
        if self.renderer is not None:
            self.renderer.close()
        super().close()
