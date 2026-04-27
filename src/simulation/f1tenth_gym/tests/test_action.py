"""Tests for the action module."""
import unittest

import gymnasium as gym
import numpy as np

from f1tenth_gym.envs.action import (
    LongitudinalActionType,
    SteerActionType,
    accl_action,
    speed_action,
    steering_angle_action,
    steering_speed_action,
    longitudinal_action_from_type,
    steer_action_from_type,
    get_action_space,
    from_single_to_multi_action_space,
)
from f1tenth_gym.envs.dynamic_models import F1TENTH_VEHICLE_PARAMETERS


class TestActionTypes(unittest.TestCase):
    """Tests for action type enums."""

    def test_longitudinal_action_types(self):
        """Test LongitudinalActionType enum values."""
        self.assertEqual(LongitudinalActionType.ACCL, 1)
        self.assertEqual(LongitudinalActionType.SPEED, 2)

    def test_steer_action_types(self):
        """Test SteerActionType enum values."""
        self.assertEqual(SteerActionType.STEERING_ANGLE, 1)
        self.assertEqual(SteerActionType.STEERING_SPEED, 2)


class TestActionFunctions(unittest.TestCase):
    """Tests for action functions."""

    def setUp(self):
        self.params = F1TENTH_VEHICLE_PARAMETERS
        self.state = np.array([0.0, 0.0, 0.1, 5.0, 0.0, 0.0, 0.0], dtype=np.float32)

    def test_accl_action(self):
        """Test direct acceleration action returns input unchanged."""
        action = 2.5
        result = accl_action(action, self.state, self.params)
        self.assertEqual(result, action)

    def test_speed_action(self):
        """Test speed action uses PID control."""
        target_speed = 10.0
        result = speed_action(target_speed, self.state, self.params)
        # Should return positive acceleration since target > current (5.0)
        self.assertGreater(result, 0)

    def test_steering_angle_action(self):
        """Test steering angle action uses PID control."""
        target_angle = 0.2
        result = steering_angle_action(target_angle, self.state, self.params)
        # Should return positive steering velocity since target > current (0.1)
        self.assertGreater(result, 0)

    def test_steering_speed_action(self):
        """Test direct steering speed action returns input unchanged."""
        action = 1.5
        result = steering_speed_action(action, self.state, self.params)
        self.assertEqual(result, action)


class TestActionFromType(unittest.TestCase):
    """Tests for action function lookup."""

    def test_longitudinal_action_from_type_accl(self):
        """Test getting acceleration action function."""
        fn = longitudinal_action_from_type(LongitudinalActionType.ACCL)
        self.assertEqual(fn, accl_action)

    def test_longitudinal_action_from_type_speed(self):
        """Test getting speed action function."""
        fn = longitudinal_action_from_type(LongitudinalActionType.SPEED)
        self.assertEqual(fn, speed_action)

    def test_longitudinal_action_from_type_invalid(self):
        """Test invalid longitudinal action type raises ValueError."""
        with self.assertRaises(ValueError):
            longitudinal_action_from_type(99)

    def test_steer_action_from_type_angle(self):
        """Test getting steering angle action function."""
        fn = steer_action_from_type(SteerActionType.STEERING_ANGLE)
        self.assertEqual(fn, steering_angle_action)

    def test_steer_action_from_type_speed(self):
        """Test getting steering speed action function."""
        fn = steer_action_from_type(SteerActionType.STEERING_SPEED)
        self.assertEqual(fn, steering_speed_action)

    def test_steer_action_from_type_invalid(self):
        """Test invalid steering action type raises ValueError."""
        with self.assertRaises(ValueError):
            steer_action_from_type(99)


class TestActionSpace(unittest.TestCase):
    """Tests for action space generation."""

    def setUp(self):
        self.params = F1TENTH_VEHICLE_PARAMETERS

    def test_action_space_accl_angle(self):
        """Test action space for acceleration + steering angle."""
        space = get_action_space(
            LongitudinalActionType.ACCL,
            SteerActionType.STEERING_ANGLE,
            self.params,
        )
        self.assertIsInstance(space, gym.spaces.Box)
        self.assertEqual(space.shape, (2,))
        # Steering limits
        self.assertAlmostEqual(space.low[0], self.params.s_min)
        self.assertAlmostEqual(space.high[0], self.params.s_max)
        # Acceleration limits
        self.assertAlmostEqual(space.low[1], -self.params.a_max)
        self.assertAlmostEqual(space.high[1], self.params.a_max)

    def test_action_space_speed_steer_speed(self):
        """Test action space for speed + steering speed."""
        space = get_action_space(
            LongitudinalActionType.SPEED,
            SteerActionType.STEERING_SPEED,
            self.params,
        )
        self.assertIsInstance(space, gym.spaces.Box)
        self.assertEqual(space.shape, (2,))
        # Steering speed limits
        self.assertAlmostEqual(space.low[0], self.params.sv_min)
        self.assertAlmostEqual(space.high[0], self.params.sv_max)
        # Velocity limits
        self.assertAlmostEqual(space.low[1], self.params.v_min)
        self.assertAlmostEqual(space.high[1], self.params.v_max)

    def test_action_space_invalid_longitudinal(self):
        """Test invalid longitudinal type raises ValueError."""
        with self.assertRaises(ValueError):
            get_action_space(99, SteerActionType.STEERING_ANGLE, self.params)

    def test_action_space_invalid_steer(self):
        """Test invalid steering type raises ValueError."""
        with self.assertRaises(ValueError):
            get_action_space(LongitudinalActionType.ACCL, 99, self.params)


class TestMultiAgentActionSpace(unittest.TestCase):
    """Tests for multi-agent action space conversion."""

    def test_single_to_multi_action_space(self):
        """Test converting single-agent to multi-agent action space."""
        single_space = gym.spaces.Box(
            low=np.array([-1.0, -2.0]),
            high=np.array([1.0, 2.0]),
            dtype=np.float32,
        )
        num_agents = 3
        multi_space = from_single_to_multi_action_space(single_space, num_agents)

        self.assertEqual(multi_space.shape, (num_agents, 2))
        for i in range(num_agents):
            np.testing.assert_array_almost_equal(multi_space.low[i], single_space.low)
            np.testing.assert_array_almost_equal(multi_space.high[i], single_space.high)
