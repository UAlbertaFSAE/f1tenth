from __future__ import annotations

from enum import IntEnum
from typing import Iterable

__all__ = [
    "ObservationType",
    "ALL_FEATURES",
    "Observation",
    "scan_space",
    "FullObservation",
    "observation_factory",
]


class ObservationType(IntEnum):
    """Type of observation returned by the environment.

    DIRECT: Full observation dict with all base fields.
    ORIGINAL: Same as DIRECT (backwards compatibility).
    FEATURES: Custom subset of features specified via config.
    KINEMATIC_STATE: Kinematic state features (x, y, delta, vx, theta).
    DYNAMIC_STATE: Dynamic state features including angular velocity and slip angle.
    FRENET_DYNAMIC_STATE: Dynamic state with separate vx/vy components.
    """

    DIRECT = 1
    ORIGINAL = 2
    FEATURES = 3
    KINEMATIC_STATE = 4
    DYNAMIC_STATE = 5
    FRENET_DYNAMIC_STATE = 6


BASE_FIELDS: tuple[str, ...] = (
    "scan",
    "std_state",
    "state",
    "collision",
    "lap_time",
    "lap_count",
    "sim_time",
    "frenet_pose",
)

DERIVED_FIELDS: tuple[str, ...] = (
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

ALL_FEATURES: tuple[str, ...] = BASE_FIELDS + DERIVED_FIELDS
_ALLOWED_FIELDS = set(ALL_FEATURES)


from .base import Observation, scan_space  # noqa: E402
from .full import FullObservation  # noqa: E402


FEATURE_PRESETS: dict[ObservationType, tuple[str, ...]] = {
    ObservationType.KINEMATIC_STATE: (
        "pose_x",
        "pose_y",
        "delta",
        "linear_vel_x",
        "pose_theta",
    ),
    ObservationType.DYNAMIC_STATE: (
        "pose_x",
        "pose_y",
        "delta",
        "linear_vel_magnitude",
        "pose_theta",
        "ang_vel_z",
        "beta",
    ),
    ObservationType.FRENET_DYNAMIC_STATE: (
        "pose_x",
        "pose_y",
        "delta",
        "linear_vel_x",
        "linear_vel_y",
        "pose_theta",
        "ang_vel_z",
        "beta",
    ),
}


def _normalize_fields(fields: Iterable[str] | None) -> tuple[str, ...]:
    if fields is None:
        raise ValueError("FullObservation requires 'features' to be specified")

    normalized = tuple(fields)
    if not normalized:
        raise ValueError("FullObservation requires at least one feature")

    invalid = next((item for item in normalized if item not in _ALLOWED_FIELDS), None)
    if invalid is not None:
        raise ValueError(f"Unknown observation feature: {invalid!r}")

    return normalized


def observation_factory(
    env,
    type: ObservationType | None = None,
    **kwargs,
) -> Observation:
    """Create an Observation instance based on the specified type.

    Args:
        env: The F110Env environment instance.
        type: Observation type to create (default: DIRECT).
        **kwargs: Additional arguments (e.g., features for FEATURES type).

    Returns:
        Configured Observation instance.
    """
    if type is None:
        obs_type = ObservationType.DIRECT
    elif isinstance(type, ObservationType):
        obs_type = type
    else:
        raise TypeError("observation_factory 'type' must be an ObservationType")

    if obs_type is ObservationType.DIRECT:
        return FullObservation(env)
    if obs_type is ObservationType.ORIGINAL:
        return FullObservation(env)
    if obs_type in FEATURE_PRESETS:
        return FullObservation(env, fields=FEATURE_PRESETS[obs_type])
    if obs_type is ObservationType.FEATURES:
        return FullObservation(env, fields=_normalize_fields(kwargs.get("features")))

    raise ValueError(f"Unsupported observation type: {obs_type}")


