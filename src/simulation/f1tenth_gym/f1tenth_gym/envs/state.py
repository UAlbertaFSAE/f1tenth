"""Runtime state containers for the simplified simulator."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class SimulationState:
    """Collection of arrays describing the multi-agent simulation."""

    state: np.ndarray
    standard_state: np.ndarray
    control_input: np.ndarray
    scans: np.ndarray
    poses: np.ndarray
    frenet: np.ndarray
    collisions: np.ndarray
    lap_counts: np.ndarray
    lap_times: np.ndarray
    lap_time_last_finish: np.ndarray
    sim_time: float
    delay_buffer: Optional[np.ndarray]
    delay_head: Optional[np.ndarray]

    @classmethod
    def allocate(
        cls,
        *,
        num_agents: int,
        state_dim: int,
        scan_size: int,
        control_dim: int,
        delay_steps: int,
    ) -> "SimulationState":
        state = np.zeros((num_agents, state_dim), dtype=np.float32)
        standard_state = np.zeros((num_agents, 7), dtype=np.float32)
        control_input = np.zeros((num_agents, control_dim), dtype=np.float32)
        scans = np.zeros((num_agents, scan_size), dtype=np.float32)
        poses = np.zeros((num_agents, 3), dtype=np.float32)
        frenet = np.zeros((num_agents, 3), dtype=np.float32)
        collisions = np.zeros((num_agents,), dtype=np.float32)
        lap_counts = np.zeros((num_agents,), dtype=np.int32)
        lap_times = np.zeros((num_agents,), dtype=np.float32)
        lap_time_last_finish = np.zeros((num_agents,), dtype=np.float32)

        if delay_steps > 0:
            delay_buffer = np.zeros((num_agents, delay_steps), dtype=np.float32)
            delay_head = np.zeros((num_agents,), dtype=np.int32)
        else:
            delay_buffer = None
            delay_head = None

        return cls(
            state=state,
            standard_state=standard_state,
            control_input=control_input,
            scans=scans,
            poses=poses,
            frenet=frenet,
            collisions=collisions,
            lap_counts=lap_counts,
            lap_times=lap_times,
            lap_time_last_finish=lap_time_last_finish,
            sim_time=0.0,
            delay_buffer=delay_buffer,
            delay_head=delay_head,
        )

    def reset(self) -> None:
        self.state.fill(0.0)
        self.standard_state.fill(0.0)
        self.control_input.fill(0.0)
        self.scans.fill(0.0)
        self.poses.fill(0.0)
        self.frenet.fill(0.0)
        self.collisions.fill(0.0)
        self.lap_counts.fill(0)
        self.lap_times.fill(0.0)
        self.lap_time_last_finish.fill(0.0)
        self.sim_time = 0.0
        if self.delay_buffer is not None and self.delay_head is not None:
            self.delay_buffer.fill(0.0)
            self.delay_head.fill(0)

    def push_delay(self, values: np.ndarray) -> np.ndarray:
        if self.delay_buffer is None or self.delay_head is None:
            return values
        if values.ndim != 1 or values.shape[0] != self.delay_buffer.shape[0]:
            raise ValueError("Values must be a 1D array with length equal to num_agents")
        delayed = np.empty_like(values)
        buffer = self.delay_buffer
        head = self.delay_head
        delay_steps = buffer.shape[1]
        for agent_index in range(values.shape[0]):
            index = head[agent_index]
            delayed[agent_index] = buffer[agent_index, index]
            buffer[agent_index, index] = values[agent_index]
            head[agent_index] = (index + 1) % delay_steps
        return delayed
