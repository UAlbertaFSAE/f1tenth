"""Tests for the integrators module."""
import unittest

import numpy as np

from f1tenth_gym.envs.integrators import (
    IntegratorType,
    euler_integration,
    rk4_integration,
    integrator_from_type,
)


class TestIntegratorType(unittest.TestCase):
    """Tests for IntegratorType enum."""

    def test_enum_values(self):
        """Test IntegratorType enum values."""
        self.assertEqual(IntegratorType.EULER, 1)
        self.assertEqual(IntegratorType.RK4, 2)

    def test_integration_fn_euler(self):
        """Test getting Euler integration function from enum."""
        fn = IntegratorType.EULER.integration_fn()
        self.assertEqual(fn, euler_integration)

    def test_integration_fn_rk4(self):
        """Test getting RK4 integration function from enum."""
        fn = IntegratorType.RK4.integration_fn()
        self.assertEqual(fn, rk4_integration)

    def test_from_type_euler(self):
        """Test from_type class method for Euler."""
        fn = IntegratorType.from_type(IntegratorType.EULER)
        self.assertEqual(fn, euler_integration)

    def test_from_type_rk4(self):
        """Test from_type class method for RK4."""
        fn = IntegratorType.from_type(IntegratorType.RK4)
        self.assertEqual(fn, rk4_integration)

    def test_from_type_invalid(self):
        """Test from_type raises TypeError for invalid input."""
        with self.assertRaises(TypeError):
            IntegratorType.from_type("euler")

    def test_integrator_from_type_helper(self):
        """Test integrator_from_type helper function."""
        fn = integrator_from_type(IntegratorType.RK4)
        self.assertEqual(fn, rk4_integration)


class TestEulerIntegration(unittest.TestCase):
    """Tests for Euler integration."""

    def test_constant_derivative(self):
        """Test Euler with constant derivative (linear motion)."""
        # dx/dt = 1 (constant velocity)
        def f(x, u, *args):
            return np.array([1.0])

        x0 = np.array([0.0])
        u = np.array([0.0])
        dt = 0.1

        x1 = euler_integration(f, x0, u, dt)
        self.assertAlmostEqual(x1[0], 0.1)

        x2 = euler_integration(f, x1, u, dt)
        self.assertAlmostEqual(x2[0], 0.2)

    def test_with_control_input(self):
        """Test Euler integration with control input."""
        # dx/dt = u (velocity equals control)
        def f(x, u, *args):
            return u

        x0 = np.array([0.0])
        u = np.array([2.0])
        dt = 0.5

        x1 = euler_integration(f, x0, u, dt)
        self.assertAlmostEqual(x1[0], 1.0)

    def test_multidimensional(self):
        """Test Euler with multidimensional state."""
        # Simple 2D motion: dx = vx*dt, dy = vy*dt
        def f(x, u, *args):
            return np.array([1.0, 2.0])  # vx=1, vy=2

        x0 = np.array([0.0, 0.0])
        u = np.array([0.0])
        dt = 0.1

        x1 = euler_integration(f, x0, u, dt)
        np.testing.assert_array_almost_equal(x1, [0.1, 0.2])


class TestRK4Integration(unittest.TestCase):
    """Tests for RK4 integration."""

    def test_constant_derivative(self):
        """Test RK4 with constant derivative (linear motion)."""
        # dx/dt = 1
        def f(x, u, *args):
            return np.array([1.0])

        x0 = np.array([0.0])
        u = np.array([0.0])
        dt = 0.1

        x1 = rk4_integration(f, x0, u, dt)
        self.assertAlmostEqual(x1[0], 0.1)

    def test_quadratic_growth(self):
        """Test RK4 with linear derivative (quadratic growth)."""
        # dx/dt = x -> x(t) = x0 * e^t
        def f(x, u, *args):
            return x

        x0 = np.array([1.0])
        u = np.array([0.0])
        dt = 0.1

        # After one step, x should be close to e^0.1 = 1.10517...
        x1 = rk4_integration(f, x0, u, dt)
        expected = np.exp(0.1)
        self.assertAlmostEqual(x1[0], expected, places=5)

    def test_rk4_more_accurate_than_euler(self):
        """Test that RK4 is more accurate than Euler for nonlinear ODE."""
        # dx/dt = x -> exact solution x(t) = e^t
        def f(x, u, *args):
            return x

        x0 = np.array([1.0])
        u = np.array([0.0])
        dt = 0.5
        t_final = 1.0
        steps = int(t_final / dt)

        # Euler integration
        x_euler = x0.copy()
        for _ in range(steps):
            x_euler = euler_integration(f, x_euler, u, dt)

        # RK4 integration
        x_rk4 = x0.copy()
        for _ in range(steps):
            x_rk4 = rk4_integration(f, x_rk4, u, dt)

        exact = np.exp(t_final)
        euler_error = abs(x_euler[0] - exact)
        rk4_error = abs(x_rk4[0] - exact)

        # RK4 should be significantly more accurate
        self.assertLess(rk4_error, euler_error)

    def test_with_extra_args(self):
        """Test that extra args are passed to the dynamics function."""
        def f(x, u, scale):
            return scale * x

        x0 = np.array([1.0])
        u = np.array([0.0])
        dt = 0.1
        scale = 2.0

        x1 = rk4_integration(f, x0, u, dt, scale)
        # dx/dt = 2x -> x(t) = e^(2t)
        expected = np.exp(2.0 * 0.1)
        self.assertAlmostEqual(x1[0], expected, places=5)
