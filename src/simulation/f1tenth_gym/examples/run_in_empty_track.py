import numpy as np


from waypoint_follow import PurePursuitPlanner
from f1tenth_gym.envs.track import Track
from f1tenth_gym.envs.observation import ObservationType
from f1tenth_gym.envs.reset import ResetStrategy
from f1tenth_gym.envs.env_config import EnvConfig, ObservationConfig, ResetConfig
import gymnasium as gym
from f1tenth_gym.envs.rendering import make_lidar_scan_callback



def main():
    """
    Demonstrate the creation of an empty map with a custom reference line.
    This is useful for testing and debugging control algorithms on standard maneuvers.
    """

    xs = np.linspace(0, 10, num=100)
    ys = np.zeros_like(xs)
    velxs = np.ones_like(xs) * 3.0

    track = Track.from_refline(x=xs, y=ys, velx=velxs)

    cfg = EnvConfig(
        map_name=track,
        observation_config=ObservationConfig(type=ObservationType.KINEMATIC_STATE),
        reset_config=ResetConfig(strategy=ResetStrategy.RL_RANDOM_STATIC),
    )

    env = gym.make(
        "f1tenth_gym:f1tenth-v0",
        config=cfg,
        render_mode="unlimited",
    )
    planner = PurePursuitPlanner(track=track, wb=0.17145 + 0.15875)

    env.unwrapped.add_render_callback(track.raceline.render_waypoints)
    env.unwrapped.add_render_callback(planner.render_lookahead_point)

    # Add lidar scan visualization
    lidar_callback = make_lidar_scan_callback("agent_0", cfg.lidar_config, color=(255, 0, 0), size=2)
    env.unwrapped.add_render_callback(lidar_callback)

    obs, info = env.reset()
    done = False
    env.render()

    while not done:
        speed, steer = planner.plan(
            obs["agent_0"]["pose_x"],
            obs["agent_0"]["pose_y"],
            obs["agent_0"]["pose_theta"],
            lookahead_distance=1.0,
            vgain=0.8,
        )
        action = np.array([[steer, speed]])
        obs, timestep, terminated, truncated, infos = env.step(action)
        done = terminated or truncated
        env.render()

    env.close()



if __name__ == "__main__":
    main()


