# MIT License

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Prototype of Utility functions and classes for simulating 2D LIDAR scans
Author: Hongrui Zheng
"""

import math
import os
import unittest

import gymnasium as gym
import numpy as np

from f1tenth_gym.envs.env_config import EnvConfig
from f1tenth_gym.envs.lidar import LiDARConfig, ScanSimulator2D


class ScanTests(unittest.TestCase):
    def setUp(self):
        # test params
        self.num_beams = 1080
        self.fov = 4.7

        self.num_test = 10
        self.test_poses = np.zeros((self.num_test, 3))
        self.test_poses[:, 2] = np.linspace(-1.0, 1.0, num=self.num_test)

        # legacy gym data
        wdir = os.path.dirname(os.path.abspath(__file__))
        self.sample_scans = np.load(f"{wdir}/legacy_scan.npz")

    def _test_map_scan(self, map_name: str, debug=False):
        scan_rng = np.random.default_rng(seed=12345)
        scan_sim = ScanSimulator2D(self.num_beams, self.fov)
        new_scan = np.empty((self.num_test, self.num_beams))
        scan_sim.set_map(map=map_name)
        # scan gen loop
        for i in range(self.num_test):
            test_pose = self.test_poses[i]
            new_scan[i, :] = scan_sim.scan(pose=test_pose, rng=scan_rng)
        diff = self.sample_scans[map_name] - new_scan
        mse = np.mean(diff**2)

        if debug:
            # plotting
            import matplotlib.pyplot as plt

            theta = np.linspace(-self.fov / 2.0, self.fov / 2.0, num=self.num_beams)
            plt.polar(theta, new_scan[1, :], ".", lw=0)
            plt.polar(theta, self.sample_scans[map_name][1, :], ".", lw=0)
            plt.show()

        self.assertLess(mse, 2.0)


    def test_map_spielberg(self, debug=False):
        self._test_map_scan("Spielberg", debug=debug)

    def test_map_monza(self, debug=False):
        self._test_map_scan("Monza", debug=debug)

    def test_map_austin(self, debug=False):
        self._test_map_scan("Austin", debug=debug)

    def test_fps(self):
        # scan fps should be greater than 500
        scan_rng = np.random.default_rng(seed=12345)
        scan_sim = ScanSimulator2D(self.num_beams, self.fov)
        scan_sim.set_map(map="Spielberg")

        import time

        start = time.time()
        for i in range(10000):
            x_test = i / 10000
            scan_sim.scan(pose=np.array([x_test, 0.0, 0.0]), rng=scan_rng)
        end = time.time()
        fps = 10000 / (end - start)

        self.assertGreater(fps, 500.0)

    def test_custom_angle_config(self):
        """Test ScanSimulator2D with custom angle_min/angle_max."""
        num_beams = 270
        angle_min = -math.radians(135)
        angle_max = math.radians(135)

        scan_sim = ScanSimulator2D(
            num_beams, fov=0, angle_min=angle_min, angle_max=angle_max
        )
        scan_sim.set_map(map="Spielberg")

        self.assertEqual(scan_sim.num_beams, num_beams)
        self.assertAlmostEqual(scan_sim.angle_min, angle_min)
        self.assertAlmostEqual(scan_sim.angle_max, angle_max)
        self.assertAlmostEqual(scan_sim.fov, angle_max - angle_min)

        expected_increment = (angle_max - angle_min) / (num_beams - 1)
        self.assertAlmostEqual(scan_sim.angle_increment, expected_increment)

        # Verify scan works and returns correct shape
        scan_rng = np.random.default_rng(seed=12345)
        scan = scan_sim.scan(pose=np.array([0.0, 0.0, 0.0]), rng=scan_rng)
        self.assertEqual(scan.shape, (num_beams,))

    def test_range_clipping(self):
        """Test that scans are clipped to min_range and max_range."""
        min_range = 0.5
        max_range = 5.0

        scan_sim = ScanSimulator2D(
            num_beams=100, fov=2.0, min_range=min_range, max_range=max_range
        )
        scan_sim.set_map(map="Spielberg")

        scan_rng = np.random.default_rng(seed=12345)
        scan = scan_sim.scan(pose=np.array([0.0, 0.0, 0.0]), rng=scan_rng)

        self.assertGreaterEqual(scan.min(), min_range)
        self.assertLessEqual(scan.max(), max_range)


class LiDARConfigTests(unittest.TestCase):
    """Tests for LiDARConfig dataclass."""

    def test_default_angles_from_fov(self):
        """Test that angle_min/angle_max are computed from field_of_view."""
        cfg = LiDARConfig(field_of_view=2.0)
        self.assertAlmostEqual(cfg.angle_min, -1.0)
        self.assertAlmostEqual(cfg.angle_max, 1.0)

    def test_explicit_angles(self):
        """Test that explicit angle_min/angle_max override field_of_view."""
        cfg = LiDARConfig(
            field_of_view=2.0,
            angle_min=-0.5,
            angle_max=1.5,
        )
        self.assertAlmostEqual(cfg.angle_min, -0.5)
        self.assertAlmostEqual(cfg.angle_max, 1.5)

    def test_angle_increment(self):
        """Test angle_increment property."""
        cfg = LiDARConfig(num_beams=101, angle_min=-1.0, angle_max=1.0)
        self.assertAlmostEqual(cfg.angle_increment, 0.02)

    def test_maximum_range_alias(self):
        """Test that maximum_range is an alias for range_max."""
        cfg = LiDARConfig(range_max=15.0)
        self.assertEqual(cfg.maximum_range, 15.0)
        self.assertEqual(cfg.maximum_range, cfg.range_max)

    def test_validation_angle_min_greater_than_max(self):
        """Test that angle_min >= angle_max raises ValueError."""
        with self.assertRaises(ValueError):
            LiDARConfig(angle_min=1.0, angle_max=0.5)

    def test_validation_range_min_greater_than_max(self):
        """Test that range_min >= range_max raises ValueError."""
        with self.assertRaises(ValueError):
            LiDARConfig(range_min=10.0, range_max=5.0)

    def test_validation_negative_range_min(self):
        """Test that negative range_min raises ValueError."""
        with self.assertRaises(ValueError):
            LiDARConfig(range_min=-1.0)

    def test_with_updates(self):
        """Test that with_updates creates a new config with updated values."""
        cfg = LiDARConfig(num_beams=100, range_max=20.0)
        updated = cfg.with_updates(num_beams=200, range_max=15.0)

        # Original unchanged
        self.assertEqual(cfg.num_beams, 100)
        self.assertEqual(cfg.range_max, 20.0)

        # Updated has new values
        self.assertEqual(updated.num_beams, 200)
        self.assertEqual(updated.range_max, 15.0)

        # Other fields preserved
        self.assertEqual(updated.enabled, cfg.enabled)
        self.assertEqual(updated.noise_std, cfg.noise_std)


class LiDARIntegrationTests(unittest.TestCase):
    """Integration tests for LiDAR config with gym environment."""

    def test_custom_lidar_in_env(self):
        """Test custom LiDAR config works in full gym environment."""
        lidar_config = LiDARConfig(
            angle_min=-math.radians(90),
            angle_max=math.radians(90),
            num_beams=180,
            range_min=0.0,
            range_max=10.0,
        )
        config = EnvConfig(
            map_name="Spielberg",
            num_agents=1,
            lidar_config=lidar_config,
        )

        env = gym.make("f1tenth_gym:f1tenth-v0", config=config)
        obs, _ = env.reset()

        scan = obs["agent_0"]["scan"]
        self.assertEqual(scan.shape, (180,))
        self.assertLessEqual(scan.max(), 10.0 + 0.001)

        env.close()
