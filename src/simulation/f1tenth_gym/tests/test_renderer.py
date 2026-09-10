import unittest

import numpy as np
from f1tenth_gym.envs.env_config import EnvConfig, ObservationConfig
from f1tenth_gym.envs.observation import ObservationType
import gymnasium as gym


class TestRenderer(unittest.TestCase):
    def test_rgb_array_render(self):
        env =  gym.make(
            "f1tenth_gym:f1tenth-v0",
            config=EnvConfig(observation_config=ObservationConfig(type=ObservationType.KINEMATIC_STATE)),
            render_mode="rgb_array",
        )
        env.reset()
        for _ in range(100):
            action = env.action_space.sample()
            env.step(action)
            frame = env.render()

            self.assertTrue(isinstance(frame, np.ndarray), "Frame is not a numpy array")
            self.assertTrue(len(frame.shape) == 3, "Frame is not a 3D array")
            self.assertTrue(frame.shape[2] == 3, "Frame does not have 3 channels")

        env.close()

        self.assertTrue(True, "rgb_array render test failed")

    def test_rgb_array_list(self):
        env = gym.make(
            "f1tenth_gym:f1tenth-v0",
            config=EnvConfig(observation_config=ObservationConfig(type=ObservationType.KINEMATIC_STATE)),
            render_mode="rgb_array_list",
        )
        env.reset()

        steps = 100
        for _ in range(steps):
            action = env.action_space.sample()
            env.step(action)

        frame_list = env.render()

        self.assertTrue(
            isinstance(frame_list, list), "the returned object is not a list of frames"
        )
        self.assertTrue(
            len(frame_list) == steps + 1,
            "the returned list does not have the correct number of frames",
        )
        self.assertTrue(
            all([isinstance(frame, np.ndarray) for frame in frame_list]),
            "not all frames are numpy arrays",
        )
        self.assertTrue(
            all([len(frame.shape) == 3 for frame in frame_list]),
            "not all frames are 3D arrays",
        )
        self.assertTrue(
            all([frame.shape[2] == 3 for frame in frame_list]),
            "not all frames have 3 channels",
        )

        env.close()


