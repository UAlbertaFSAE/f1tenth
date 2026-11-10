from __future__ import annotations

from typing import Callable

import gymnasium as gym
import numpy as np

from .base import Observation, scan_space

__all__ = ["FullObservation"]

_BASE_FIELDS: tuple[str, ...] = (
    "scan",
    "std_state",
    "state",
    "collision",
    "lap_time",
    "lap_count",
    "sim_time",
    "frenet_pose",
)

_DERIVED_FIELDS: tuple[str, ...] = (
    "pose_x",
    "pose_y",
    "pose_theta",
    "linear_vel_x",
    "linear_vel_y",
    "linear_vel_magnitude",
    "ang_vel_z",
    "delta",
    "beta",
)

_ALL_FIELDS = set(_BASE_FIELDS) | set(_DERIVED_FIELDS)


def _scalar_box(low: float, high: float) -> gym.Space:
    return gym.spaces.Box(low=low, high=high, shape=(), dtype=np.float32)


_FIELD_SPACE_BUILDERS: dict[str, Callable[[object, float], gym.Space]] = {
    "scan": lambda sim, limit: scan_space(sim),
    "std_state": lambda sim, limit: gym.spaces.Box(
        low=-limit, high=limit, shape=(7,), dtype=np.float32
    ),
    "state": lambda sim, limit: gym.spaces.Box(
        low=-limit, high=limit, shape=(sim.state_dim,), dtype=np.float32
    ),
    "collision": lambda sim, limit: _scalar_box(0.0, 1.0),
    "lap_time": lambda sim, limit: _scalar_box(0.0, limit),
    "lap_count": lambda sim, limit: _scalar_box(0.0, limit),
    "sim_time": lambda sim, limit: _scalar_box(0.0, limit),
    "frenet_pose": lambda sim, limit: gym.spaces.Box(
        low=-limit, high=limit, shape=(3,), dtype=np.float32
    ),
    "pose_x": lambda sim, limit: _scalar_box(-limit, limit),
    "pose_y": lambda sim, limit: _scalar_box(-limit, limit),
    "pose_theta": lambda sim, limit: _scalar_box(-limit, limit),
    "linear_vel_x": lambda sim, limit: _scalar_box(-limit, limit),
    "linear_vel_y": lambda sim, limit: _scalar_box(-limit, limit),
    "linear_vel_magnitude": lambda sim, limit: _scalar_box(0.0, limit),
    "ang_vel_z": lambda sim, limit: _scalar_box(-limit, limit),
    "delta": lambda sim, limit: _scalar_box(-limit, limit),
    "beta": lambda sim, limit: _scalar_box(-limit, limit),
}


class FullObservation(Observation):
    """Observation provider that exposes simulator state with optional feature filtering."""

    def __init__(self, env, fields: tuple[str, ...] | None = None):
        super().__init__(env)
        if fields is None:
            self._fields: tuple[str, ...] | None = None
        else:
            normalized = tuple(fields)
            if not normalized:
                raise ValueError("FullObservation requires at least one field")
            unknown = next((item for item in normalized if item not in _ALL_FIELDS), None)
            if unknown is not None:
                raise ValueError(f"Unknown observation field: {unknown!r}")
            self._fields = normalized

    def _selected_fields(self) -> tuple[str, ...]:
        if self._fields is None:
            if self.env.unwrapped.compute_frenet:
                return _BASE_FIELDS
            return tuple(field for field in _BASE_FIELDS if field != "frenet_pose")
        if "frenet_pose" in self._fields and not self.env.unwrapped.compute_frenet:
            raise ValueError("frenet_pose requested but environment does not compute the Frenet frame")
        return self._fields

    def space(self) -> gym.Space:
        sim = self._sim
        limit = 1e30
        fields = self._selected_fields()

        agent_spaces: dict[str, gym.Space] = {}
        for agent_id in self.env.unwrapped.agent_ids:
            feature_spaces = {
                field: _FIELD_SPACE_BUILDERS[field](sim, limit)
                for field in fields
            }
            agent_spaces[agent_id] = gym.spaces.Dict(feature_spaces)
        return gym.spaces.Dict(agent_spaces)

    def observe(self):
        sim = self._sim
        state = self._state
        fields = self._selected_fields()
        needs_derived = any(field in _DERIVED_FIELDS for field in fields)

        beam_count = sim.scan_num_beams
        observations: dict[str, dict[str, np.ndarray]] = {}

        for idx, agent_id in enumerate(self.env.unwrapped.agent_ids):
            scan = (
                state.scans[idx, :beam_count]
                if beam_count > 0
                else np.empty((0,), dtype=np.float32)
            )
            std_state = state.standard_state[idx]
            base_values: dict[str, np.ndarray] = {
                "scan": scan.astype(np.float32, copy=False),
                "std_state": std_state.astype(np.float32),
                "state": state.state[idx].astype(np.float32),
                "collision": np.asarray(state.collisions[idx], dtype=np.float32),
                "lap_time": np.asarray(self.env.unwrapped.lap_times[idx], dtype=np.float32),
                "lap_count": np.asarray(self.env.unwrapped.lap_counts[idx], dtype=np.float32),
                "sim_time": np.asarray(self.env.unwrapped.sim_time, dtype=np.float32),
            }
            if self.env.unwrapped.compute_frenet:
                base_values["frenet_pose"] = state.frenet[idx].astype(np.float32)

            derived_values: dict[str, np.ndarray] = {}
            if needs_derived:
                speed = std_state[3]
                beta = std_state[6]
                vx = speed * np.cos(beta)
                vy = speed * np.sin(beta)
                derived_values = {
                    "pose_x": np.asarray(std_state[0], dtype=np.float32),
                    "pose_y": np.asarray(std_state[1], dtype=np.float32),
                    "pose_theta": np.asarray(std_state[4], dtype=np.float32),
                    "linear_vel_x": np.asarray(vx, dtype=np.float32),
                    "linear_vel_y": np.asarray(vy, dtype=np.float32),
                    "linear_vel_magnitude": np.asarray(speed, dtype=np.float32),
                    "ang_vel_z": np.asarray(std_state[5], dtype=np.float32),
                    "delta": np.asarray(std_state[2], dtype=np.float32),
                    "beta": np.asarray(beta, dtype=np.float32),
                }

            agent_obs: dict[str, np.ndarray] = {}
            for field in fields:
                if field in base_values:
                    agent_obs[field] = base_values[field]
                else:
                    agent_obs[field] = derived_values[field]
            observations[agent_id] = agent_obs

        return observations



