"""Tests for the state module."""
import unittest

import numpy as np

from f1tenth_gym.envs.state import SimulationState


class TestSimulationStateAllocate(unittest.TestCase):
    """Tests for SimulationState.allocate class method."""

    def test_allocate_basic(self):
        """Test basic allocation with default parameters."""
        state = SimulationState.allocate(
            num_agents=2,
            state_dim=7,
            scan_size=1080,
            control_dim=2,
            delay_steps=0,
        )

        self.assertEqual(state.state.shape, (2, 7))
        self.assertEqual(state.standard_state.shape, (2, 7))
        self.assertEqual(state.control_input.shape, (2, 2))
        self.assertEqual(state.scans.shape, (2, 1080))
        self.assertEqual(state.poses.shape, (2, 3))
        self.assertEqual(state.frenet.shape, (2, 3))
        self.assertEqual(state.collisions.shape, (2,))
        self.assertEqual(state.lap_counts.shape, (2,))
        self.assertEqual(state.lap_times.shape, (2,))
        self.assertEqual(state.lap_time_last_finish.shape, (2,))
        self.assertEqual(state.sim_time, 0.0)
        self.assertIsNone(state.delay_buffer)
        self.assertIsNone(state.delay_head)

    def test_allocate_with_delay(self):
        """Test allocation with delay buffer."""
        state = SimulationState.allocate(
            num_agents=3,
            state_dim=7,
            scan_size=100,
            control_dim=2,
            delay_steps=5,
        )

        self.assertIsNotNone(state.delay_buffer)
        self.assertIsNotNone(state.delay_head)
        self.assertEqual(state.delay_buffer.shape, (3, 5))
        self.assertEqual(state.delay_head.shape, (3,))

    def test_allocate_single_agent(self):
        """Test allocation for single agent."""
        state = SimulationState.allocate(
            num_agents=1,
            state_dim=10,
            scan_size=500,
            control_dim=2,
            delay_steps=0,
        )

        self.assertEqual(state.state.shape, (1, 10))
        self.assertEqual(state.scans.shape, (1, 500))

    def test_allocate_dtypes(self):
        """Test that arrays have correct dtypes."""
        state = SimulationState.allocate(
            num_agents=2,
            state_dim=7,
            scan_size=100,
            control_dim=2,
            delay_steps=3,
        )

        self.assertEqual(state.state.dtype, np.float32)
        self.assertEqual(state.standard_state.dtype, np.float32)
        self.assertEqual(state.scans.dtype, np.float32)
        self.assertEqual(state.collisions.dtype, np.float32)
        self.assertEqual(state.lap_counts.dtype, np.int32)
        self.assertEqual(state.delay_buffer.dtype, np.float32)
        self.assertEqual(state.delay_head.dtype, np.int32)


class TestSimulationStateReset(unittest.TestCase):
    """Tests for SimulationState.reset method."""

    def test_reset_clears_arrays(self):
        """Test that reset clears all arrays to zero."""
        state = SimulationState.allocate(
            num_agents=2,
            state_dim=7,
            scan_size=100,
            control_dim=2,
            delay_steps=3,
        )

        # Modify some values
        state.state[0, 0] = 10.0
        state.scans[1, 50] = 5.0
        state.lap_counts[0] = 3
        state.sim_time = 100.0
        state.delay_buffer[0, 0] = 1.0
        state.delay_head[0] = 2

        # Reset
        state.reset()

        # Check all values are zero
        self.assertTrue(np.all(state.state == 0.0))
        self.assertTrue(np.all(state.scans == 0.0))
        self.assertTrue(np.all(state.lap_counts == 0))
        self.assertEqual(state.sim_time, 0.0)
        self.assertTrue(np.all(state.delay_buffer == 0.0))
        self.assertTrue(np.all(state.delay_head == 0))

    def test_reset_without_delay(self):
        """Test reset works when delay buffer is None."""
        state = SimulationState.allocate(
            num_agents=2,
            state_dim=7,
            scan_size=100,
            control_dim=2,
            delay_steps=0,
        )

        state.state[0, 0] = 10.0
        state.reset()

        self.assertTrue(np.all(state.state == 0.0))


class TestSimulationStatePushDelay(unittest.TestCase):
    """Tests for SimulationState.push_delay method."""

    def test_push_delay_no_buffer(self):
        """Test push_delay returns values unchanged when no buffer."""
        state = SimulationState.allocate(
            num_agents=2,
            state_dim=7,
            scan_size=100,
            control_dim=2,
            delay_steps=0,
        )

        values = np.array([1.0, 2.0], dtype=np.float32)
        result = state.push_delay(values)

        np.testing.assert_array_equal(result, values)

    def test_push_delay_fifo(self):
        """Test push_delay implements FIFO buffer correctly."""
        state = SimulationState.allocate(
            num_agents=1,
            state_dim=7,
            scan_size=100,
            control_dim=2,
            delay_steps=3,
        )

        # Push values and check delayed output
        # Initial buffer is all zeros
        result1 = state.push_delay(np.array([1.0]))
        self.assertEqual(result1[0], 0.0)  # Returns old value (0)

        result2 = state.push_delay(np.array([2.0]))
        self.assertEqual(result2[0], 0.0)  # Still 0

        result3 = state.push_delay(np.array([3.0]))
        self.assertEqual(result3[0], 0.0)  # Still 0

        # Now the buffer should start returning pushed values
        result4 = state.push_delay(np.array([4.0]))
        self.assertEqual(result4[0], 1.0)  # Returns value pushed 3 steps ago

        result5 = state.push_delay(np.array([5.0]))
        self.assertEqual(result5[0], 2.0)

    def test_push_delay_multi_agent(self):
        """Test push_delay works for multiple agents."""
        state = SimulationState.allocate(
            num_agents=2,
            state_dim=7,
            scan_size=100,
            control_dim=2,
            delay_steps=2,
        )

        result1 = state.push_delay(np.array([1.0, 10.0]))
        np.testing.assert_array_equal(result1, [0.0, 0.0])

        result2 = state.push_delay(np.array([2.0, 20.0]))
        np.testing.assert_array_equal(result2, [0.0, 0.0])

        result3 = state.push_delay(np.array([3.0, 30.0]))
        np.testing.assert_array_equal(result3, [1.0, 10.0])

    def test_push_delay_invalid_shape(self):
        """Test push_delay raises error for invalid input shape."""
        state = SimulationState.allocate(
            num_agents=2,
            state_dim=7,
            scan_size=100,
            control_dim=2,
            delay_steps=3,
        )

        with self.assertRaises(ValueError):
            state.push_delay(np.array([1.0, 2.0, 3.0]))  # Wrong length

        with self.assertRaises(ValueError):
            state.push_delay(np.array([[1.0, 2.0]]))  # Wrong dims
