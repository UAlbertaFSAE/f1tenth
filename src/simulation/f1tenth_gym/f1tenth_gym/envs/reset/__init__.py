from __future__ import annotations

from abc import ABC, abstractmethod
from enum import IntEnum
from typing import Callable, Optional

import numpy as np

from ..track import Track

__all__ = ["ResetFn", "ResetStrategy", "make_reset_fn", "GridResetFn", "AllTrackResetFn", "AllMapResetFn"]


class ResetFn(ABC):
    """Abstract base class for episode reset functions.

    Implementations generate initial poses for agents at episode start.
    """

    @abstractmethod
    def sample(self, rng: np.random.Generator) -> np.ndarray:
        """Return initial poses for all agents.

        Args:
            rng: numpy random number generator for reproducibility.

        Returns:
            Array of shape (num_agents, 3) with [x, y, theta] per agent.
        """


class ResetStrategy(IntEnum):
    """Strategy for placing agents at episode reset.

    RL_GRID_STATIC: Grid formation near start line, fixed order.
    RL_RANDOM_STATIC: Random position along track, fixed order.
    RL_GRID_RANDOM: Grid formation near start line, shuffled order.
    RL_RANDOM_RANDOM: Random position along track, shuffled order.
    MAP_RANDOM_STATIC: Random position on map free space.
    """

    RL_GRID_STATIC = 1
    RL_RANDOM_STATIC = 2
    RL_GRID_RANDOM = 3
    RL_RANDOM_RANDOM = 4
    MAP_RANDOM_STATIC = 5


def _rl_reset_factory(
    *,
    builder: Callable[..., ResetFn],
    shuffle: bool,
    move_laterally: bool,
) -> Callable[[Track, int, dict], ResetFn]:
    def factory(track: Track, num_agents: int, kwargs: dict) -> ResetFn:
        return builder(
            reference_line=track.raceline,
            num_agents=num_agents,
            shuffle=shuffle,
            move_laterally=move_laterally,
            **kwargs,
        )

    return factory


def _map_reset_factory(
    *,
    shuffle: bool,
    move_laterally: bool,
) -> Callable[[Track, int, dict], ResetFn]:
    def factory(track: Track, num_agents: int, kwargs: dict) -> ResetFn:
        return AllMapResetFn(
            track=track,
            num_agents=num_agents,
            shuffle=shuffle,
            move_laterally=move_laterally,
            **kwargs,
        )

    return factory







def make_reset_fn(
    track: Track,
    num_agents: int,
    type: Optional[ResetStrategy] = None,
    **kwargs,
) -> ResetFn:
    """Factory for constructing a reset function based on the chosen strategy."""

    strategy = type or ResetStrategy.RL_GRID_STATIC
    if not isinstance(strategy, ResetStrategy):
        raise TypeError("type must be a ResetStrategy")

    try:
        builder = _RESET_BUILDERS[strategy]
    except KeyError as exc:
        raise ValueError(f"Unsupported reset strategy: {strategy}") from exc

    return builder(track, num_agents, kwargs)


from .masked_reset import GridResetFn, AllTrackResetFn
from .map_reset import AllMapResetFn

_RESET_BUILDERS: dict[ResetStrategy, Callable[[Track, int, dict], ResetFn]] = {
    ResetStrategy.RL_GRID_STATIC: _rl_reset_factory(
        builder=GridResetFn,
        shuffle=False,
        move_laterally=False,
    ),
    ResetStrategy.RL_RANDOM_STATIC: _rl_reset_factory(
        builder=AllTrackResetFn,
        shuffle=False,
        move_laterally=False,
    ),
    ResetStrategy.RL_GRID_RANDOM: _rl_reset_factory(
        builder=GridResetFn,
        shuffle=True,
        move_laterally=False,
    ),
    ResetStrategy.RL_RANDOM_RANDOM: _rl_reset_factory(
        builder=AllTrackResetFn,
        shuffle=True,
        move_laterally=False,
    ),
    ResetStrategy.MAP_RANDOM_STATIC: _map_reset_factory(
        shuffle=False,
        move_laterally=True,
    ),
}
