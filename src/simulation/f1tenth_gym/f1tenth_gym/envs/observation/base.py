from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

import gymnasium as gym
import numpy as np

if TYPE_CHECKING:
    from ..simulator import F110Simulator

__all__ = ["Observation", "scan_space"]

class Observation(ABC):
    """Base class for environment observations."""

    def __init__(self, env):
        self.env = env

    @abstractmethod
    def space(self) -> gym.Space:
        """Return the Gymnasium space describing this observation."""

    @abstractmethod
    def observe(self):
        """Compute the current observation."""

    @property
    def _sim(self) -> "F110Simulator":
        return self.env.unwrapped.sim

    @property
    def _state(self):
        return self._sim.state

def scan_space(sim: "F110Simulator") -> gym.spaces.Box:
    """Build a scan space that adapts to the active LiDAR configuration."""

    beam_count = sim.scan_num_beams
    max_range = sim.scan_max_range if sim.scan_enabled else 1.0
    shape = (beam_count,) if beam_count > 0 else (0,)
    low = 0.0 if beam_count > 0 else np.array([], dtype=np.float32)
    high = max_range if beam_count > 0 else np.array([], dtype=np.float32)
    return gym.spaces.Box(low=low, high=high, shape=shape, dtype=np.float32)
