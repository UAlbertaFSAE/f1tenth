"""Typed configuration structures for the simplified F1TENTH gym environment."""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import IntEnum
from typing import TYPE_CHECKING, Any, Callable, Optional

import numpy as np

from .integrators import IntegratorType
from .dynamic_models import (
    DynamicModel,
    VehicleParameters,
    F1TENTH_VEHICLE_PARAMETERS,
)
from .action import LongitudinalActionType, SteerActionType
from .observation import ObservationType
from .reset import ResetStrategy
from .lidar import LiDARConfig
from .collision_models import CollisionCheckMode

if TYPE_CHECKING:
    from .track import Track

# Type aliases for callables used throughout the simulator
DynamicsFn = Callable[[np.ndarray, np.ndarray, np.ndarray], np.ndarray]
StandardStateFn = Callable[[np.ndarray], np.ndarray]
IntegratorFn = Callable[[DynamicsFn, np.ndarray, np.ndarray, float, np.ndarray], np.ndarray]


class LoopCounterMode(IntEnum):
    """Mode for counting completed laps.

    TOGGLE: Uses start/finish line crossing detection.
    FRENET_BASED: Uses Frenet frame progress along centerline.
    WINDING_ANGLE: Uses cumulative angle around track center.
    """

    TOGGLE = 1
    FRENET_BASED = 2
    WINDING_ANGLE = 3


@dataclass(frozen=True)
class ControlConfig:
    """Configuration for vehicle control inputs.

    Attributes:
        longitudinal_mode: How longitudinal control is interpreted (speed or acceleration).
        steering_mode: How steering control is interpreted (angle or angular velocity).
        steer_delay_steps: Number of timesteps to delay steering commands.
    """

    longitudinal_mode: LongitudinalActionType = LongitudinalActionType.SPEED
    steering_mode: SteerActionType = SteerActionType.STEERING_ANGLE
    steer_delay_steps: int = 0

    def __post_init__(self) -> None:
        if self.steer_delay_steps < 0:
            raise ValueError(f"steer_delay_steps must be >= 0, got {self.steer_delay_steps}")

    def with_updates(self, **changes: Any) -> "ControlConfig":
        return replace(self, **changes)


@dataclass(frozen=True)
class SimulationConfig:
    """Configuration for physics simulation.

    Attributes:
        timestep: Simulation timestep in seconds.
        integrator_timestep: Integration timestep (can be smaller than timestep).
        integrator: Numerical integration method (Euler or RK4).
        dynamics_model: Vehicle dynamics model to use.
        loop_counter: Method for counting completed laps.
        compute_frenet_frame: Whether to compute Frenet frame coordinates.
        max_laps: Maximum laps before episode ends (None for infinite).
    """

    timestep: float = 0.01
    integrator_timestep: float = 0.01
    integrator: IntegratorType = IntegratorType.RK4
    dynamics_model: DynamicModel = DynamicModel.ST
    loop_counter: LoopCounterMode = LoopCounterMode.FRENET_BASED
    compute_frenet_frame: bool = True
    max_laps: Optional[int] = 1

    def __post_init__(self) -> None:
        if self.timestep <= 0:
            raise ValueError(f"timestep must be > 0, got {self.timestep}")
        if self.integrator_timestep <= 0:
            raise ValueError(f"integrator_timestep must be > 0, got {self.integrator_timestep}")
        if self.max_laps is not None and self.max_laps < 1:
            raise ValueError(f"max_laps must be >= 1 or None, got {self.max_laps}")

    def with_updates(self, **changes: Any) -> "SimulationConfig":
        updated = replace(self, **changes)
        if updated.loop_counter is LoopCounterMode.FRENET_BASED and not updated.compute_frenet_frame:
            updated = replace(updated, compute_frenet_frame=True)
        return updated


@dataclass(frozen=True)
class ObservationConfig:
    """Configuration for environment observations.

    Attributes:
        type: Observation format type.
        features: Specific features to include (None for all).
    """

    type: ObservationType = ObservationType.DIRECT
    features: Optional[tuple[str, ...]] = None

    def with_updates(self, **changes: Any) -> "ObservationConfig":
        return replace(self, **changes)


@dataclass(frozen=True)
class ResetConfig:
    """Configuration for episode reset behavior.

    Attributes:
        strategy: Reset strategy for initial agent positions.
    """

    strategy: ResetStrategy = ResetStrategy.RL_GRID_STATIC

    def with_updates(self, **changes: Any) -> "ResetConfig":
        return replace(self, **changes)


@dataclass(frozen=True)
class EnvConfig:
    """Main configuration for the F1TENTH environment.

    Attributes:
        seed: Random seed for reproducibility.
        map_name: Track name, path, or Track instance.
        map_scale: Scale factor for the map.
        params: Vehicle physical parameters.
        num_agents: Number of agents in the environment.
        ego_index: Index of the ego agent (0-indexed).
        control_config: Control input configuration.
        simulation_config: Physics simulation configuration.
        observation_config: Observation space configuration.
        reset_config: Episode reset configuration.
        lidar_config: LiDAR sensor configuration.
        collision_check: Collision detection mode.
        render_enabled: Whether rendering is enabled.
    """

    seed: int = 12345
    map_name: "Track | str" = "Spielberg"
    map_scale: float = 1.0
    params: VehicleParameters = F1TENTH_VEHICLE_PARAMETERS
    num_agents: int = 1
    ego_index: int = 0
    control_config: ControlConfig = field(default_factory=ControlConfig)
    simulation_config: SimulationConfig = field(default_factory=SimulationConfig)
    observation_config: ObservationConfig = field(default_factory=ObservationConfig)
    reset_config: ResetConfig = field(default_factory=ResetConfig)
    lidar_config: LiDARConfig = field(default_factory=LiDARConfig)
    collision_check: CollisionCheckMode = CollisionCheckMode.LIDAR_SCAN
    render_enabled: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.params, VehicleParameters):
            raise TypeError("params must be a VehicleParameters instance")

        object.__setattr__(self, "seed", int(self.seed))
        object.__setattr__(self, "map_scale", float(self.map_scale))
        object.__setattr__(self, "num_agents", int(self.num_agents))
        object.__setattr__(self, "ego_index", int(self.ego_index))
        object.__setattr__(self, "render_enabled", bool(self.render_enabled))

        # Validate numeric constraints
        if self.map_scale <= 0:
            raise ValueError(f"map_scale must be > 0, got {self.map_scale}")
        if self.num_agents < 1:
            raise ValueError(f"num_agents must be >= 1, got {self.num_agents}")
        if not (0 <= self.ego_index < self.num_agents):
            raise ValueError(
                f"ego_index must be in range [0, num_agents), got {self.ego_index} with num_agents={self.num_agents}"
            )

        control_cfg = self.control_config
        if not isinstance(control_cfg, ControlConfig):
            raise TypeError("control must be a ControlConfig instance")

        simulation_cfg = self.simulation_config
        if not isinstance(simulation_cfg, SimulationConfig):
            raise TypeError("simulation must be a SimulationConfig instance")

        observation_cfg = self.observation_config
        if not isinstance(observation_cfg, ObservationConfig):
            raise TypeError("observation must be an ObservationConfig instance")

        reset_cfg = self.reset_config
        if not isinstance(reset_cfg, ResetConfig):
            raise TypeError("reset must be a ResetConfig instance")

        lidar_cfg = self.lidar_config
        if not isinstance(lidar_cfg, LiDARConfig):
            raise TypeError("lidar must be a LiDARConfig instance")

        if (
            simulation_cfg.loop_counter is LoopCounterMode.FRENET_BASED
            and not simulation_cfg.compute_frenet_frame
        ):
            simulation_cfg = simulation_cfg.with_updates(compute_frenet_frame=True)

        object.__setattr__(self, "control_config", control_cfg)
        object.__setattr__(self, "simulation_config", simulation_cfg)
        object.__setattr__(self, "observation_config", observation_cfg)
        object.__setattr__(self, "reset_config", reset_cfg)
        object.__setattr__(self, "lidar_config", lidar_cfg)

    def with_updates(self, **changes: Any) -> "EnvConfig":
        return replace(self, **changes)