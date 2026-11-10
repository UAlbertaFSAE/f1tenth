import unittest
import math
from dataclasses import fields

import numpy as np
import gymnasium as gym

from f1tenth_gym.envs.env_config import EnvConfig

class TestUtilities(unittest.TestCase):
    def test_env_config_param_override(self):
        """Verify parameter overrides through EnvConfig propagate into the simulator."""
        base_cfg = EnvConfig()
        custom_cfg = base_cfg.with_updates(params=base_cfg.params.with_updates(mu=1.0))

        default_env = gym.make("f1tenth_gym:f1tenth-v0", config=base_cfg)
        custom_env = gym.make("f1tenth_gym:f1tenth-v0", config=custom_cfg)

        default_params = default_env.unwrapped.vehicle_params
        custom_params = custom_env.unwrapped.vehicle_params
        for field in fields(default_params):
            default_val = getattr(default_params, field.name)
            custom_val = getattr(custom_params, field.name)
            if field.name == "mu":
                self.assertNotEqual(default_val, custom_val, "mu should be different")
            elif isinstance(default_val, (float, np.floating)) and isinstance(custom_val, (float, np.floating)) and math.isnan(default_val) and math.isnan(custom_val):
                continue
            else:
                self.assertEqual(default_val, custom_val, f"{field.name} should be the same")

        default_env.close()
        custom_env.close()

