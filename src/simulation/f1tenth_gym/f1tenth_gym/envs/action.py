from enum import IntEnum

import gymnasium as gym
import numpy as np
from .dynamic_models import VehicleParameters, pid_steer, pid_accl

class LongitudinalActionType(IntEnum):
    """Longitudinal control input mode.

    ACCL: Direct acceleration command in m/s^2.
    SPEED: Target speed command in m/s (uses PID control).
    """

    ACCL = 1
    SPEED = 2


class SteerActionType(IntEnum):
    """Steering control input mode.

    STEERING_ANGLE: Target steering angle in radians (uses PID control).
    STEERING_SPEED: Direct steering velocity in rad/s.
    """

    STEERING_ANGLE = 1
    STEERING_SPEED = 2

def accl_action(action: float, state: np.ndarray, params: VehicleParameters) -> float:
    """Direct acceleration control"""
    return action

def speed_action(action: float, state: np.ndarray, params: VehicleParameters) -> float:
    """Speed control using PID"""
    return pid_accl(
        action,
        state[3],  # current velocity
        params.a_max,
        params.v_max,
        params.v_min,
    )

def steering_angle_action(action: float, state: np.ndarray, params: VehicleParameters) -> float:
    """Steering angle control using PID"""
    return pid_steer(
        action,
        state[2],  # current steering angle
        params.sv_max,
    )

def steering_speed_action(action: float, state: np.ndarray, params: VehicleParameters) -> float:
    """Direct steering velocity control"""
    return action

def longitudinal_action_from_type(action_type: LongitudinalActionType):
    """Get longitudinal action function from type"""
    if action_type == LongitudinalActionType.ACCL:
        return accl_action
    elif action_type == LongitudinalActionType.SPEED:
        return speed_action
    else:
        raise ValueError(f"Unknown longitudinal action type: {action_type}")

def steer_action_from_type(action_type: SteerActionType):
    """Get steering action function from type"""
    if action_type == SteerActionType.STEERING_ANGLE:
        return steering_angle_action
    elif action_type == SteerActionType.STEERING_SPEED:
        return steering_speed_action
    else:
        raise ValueError(f"Unknown steering action type: {action_type}")

def get_action_space(
    longitudinal_type: LongitudinalActionType,
    steer_type: SteerActionType,
    params: VehicleParameters
) -> gym.Space:
    """Get the action space for the given action types"""
    # Get limits based on action types
    if longitudinal_type == LongitudinalActionType.ACCL:
        long_low, long_high = -params.a_max, params.a_max
    elif longitudinal_type == LongitudinalActionType.SPEED:
        long_low, long_high = params.v_min, params.v_max
    else:
        raise ValueError(f"Unknown longitudinal action type: {longitudinal_type}")
    
    if steer_type == SteerActionType.STEERING_ANGLE:
        steer_low, steer_high = params.s_min, params.s_max
    elif steer_type == SteerActionType.STEERING_SPEED:
        steer_low, steer_high = params.sv_min, params.sv_max
    else:
        raise ValueError(f"Unknown steering action type: {steer_type}")
    
    low = np.array([steer_low, long_low]).astype(np.float32)
    high = np.array([steer_high, long_high]).astype(np.float32)
    
    return gym.spaces.Box(low=low, high=high, shape=(2,), dtype=np.float32)

def from_single_to_multi_action_space(
    single_agent_action_space: gym.spaces.Box, num_agents: int
) -> gym.spaces.Box:
    """Convert single agent action space to multi-agent action space"""
    return gym.spaces.Box(
        low=single_agent_action_space.low[None].repeat(num_agents, 0),
        high=single_agent_action_space.high[None].repeat(num_agents, 0),
        shape=(num_agents, single_agent_action_space.shape[0]),
        dtype=np.float32,
    )
