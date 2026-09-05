import unittest

import gymnasium as gym
import numpy as np

from f1tenth_gym.envs.env_config import EnvConfig, ObservationConfig
from f1tenth_gym.envs.observation import (
    observation_factory,
    ObservationType,
)


class TestObservationInterface(unittest.TestCase):
    def test_direct_default_fields(self):
        env = gym.make(
            "f1tenth_gym:f1tenth-v0",
            config=EnvConfig(
                observation_config=ObservationConfig(type=ObservationType.DIRECT)
            ),
        )

        obs, _ = env.reset()
        direct_obs = observation_factory(env, type=ObservationType.DIRECT)
        space = direct_obs.space()
        sample = direct_obs.observe()

        expected_fields = {
            "scan",
            "std_state",
            "state",
            "collision",
            "lap_time",
            "lap_count",
            "sim_time",
        }
        if env.unwrapped.compute_frenet:
            expected_fields.add("frenet_pose")

        for agent_id in env.unwrapped.agent_ids:
            agent_space = space.spaces[agent_id]
            self.assertTrue(isinstance(agent_space, gym.spaces.Dict))
            self.assertEqual(set(agent_space.spaces.keys()), expected_fields)
            self.assertEqual(set(sample[agent_id].keys()), expected_fields)
            self.assertTrue(agent_space.contains(sample[agent_id]))
            self.assertIn(agent_id, obs)
        env.close()

    def test_feature_subset(self):
        features = ("pose_x", "pose_y", "pose_theta")
        env = gym.make(
            "f1tenth_gym:f1tenth-v0",
            config=EnvConfig(
                observation_config=ObservationConfig(
                    type=ObservationType.FEATURES,
                    features=features,
                )
            ),
        )

        obs, _ = env.reset()
        obs, _, _, _, _ = env.step(env.action_space.sample())

        for agent_id in env.unwrapped.agent_ids:
            self.assertEqual(set(obs[agent_id].keys()), set(features))
            self.assertTrue(all(isinstance(value, np.ndarray) for value in obs[agent_id].values()))

        sim_state = env.unwrapped.sim.state
        for idx, agent_id in enumerate(env.unwrapped.agent_ids):
            pose_x, pose_y, pose_theta = sim_state.poses[idx]
            agent_obs = obs[agent_id]
            observed = np.asarray([
                agent_obs["pose_x"],
                agent_obs["pose_y"],
                agent_obs["pose_theta"],
            ], dtype=np.float32)
            expected = np.asarray([pose_x, pose_y, pose_theta], dtype=np.float32)
            self.assertTrue(np.allclose(observed, expected))
        env.close()

    def test_presets_match_expected_fields(self):
        preset_expectations = {
            ObservationType.KINEMATIC_STATE: {"pose_x", "pose_y", "delta", "linear_vel_x", "pose_theta"},
            ObservationType.DYNAMIC_STATE: {
                "pose_x",
                "pose_y",
                "delta",
                "linear_vel_magnitude",
                "pose_theta",
                "ang_vel_z",
                "beta",
            },
            ObservationType.FRENET_DYNAMIC_STATE: {
                "pose_x",
                "pose_y",
                "delta",
                "linear_vel_x",
                "linear_vel_y",
                "pose_theta",
                "ang_vel_z",
                "beta",
            },
        }

        for obs_type, expected in preset_expectations.items():
            env = gym.make(
                "f1tenth_gym:f1tenth-v0",
                config=EnvConfig(
                    observation_config=ObservationConfig(type=obs_type)
                ),
            )
            obs_impl = observation_factory(env, type=obs_type)
            space = obs_impl.space()
            observation = obs_impl.observe()

            for agent_id in env.unwrapped.agent_ids:
                self.assertEqual(set(space.spaces[agent_id].spaces.keys()), expected)
                self.assertEqual(set(observation[agent_id].keys()), expected)
                self.assertTrue(space.spaces[agent_id].contains(observation[agent_id]))
            env.close()

    def test_invalid_feature_name(self):
        env = gym.make("f1tenth_gym:f1tenth-v0", config=EnvConfig())
        with self.assertRaises(ValueError):
            observation_factory(
                env,
                type=ObservationType.FEATURES,
                features=("unknown_feature",),
            )
        env.close()

    def test_invalid_observation_type(self):
        env = gym.make("f1tenth_gym:f1tenth-v0", config=EnvConfig())
        with self.assertRaises(TypeError):
            observation_factory(env, type="unsupported")
        env.close()

    def test_space_contains_observation(self):
        obs_types = [
            ObservationType.DIRECT,
            ObservationType.KINEMATIC_STATE,
            ObservationType.DYNAMIC_STATE,
        ]
        env = gym.make("f1tenth_gym:f1tenth-v0", config=EnvConfig())
        env.reset()

        for obs_type in obs_types:
            obs_impl = observation_factory(env, type=obs_type)
            space = obs_impl.space()
            observation = obs_impl.observe()
            self.assertTrue(space.contains(observation))
        env.close()

