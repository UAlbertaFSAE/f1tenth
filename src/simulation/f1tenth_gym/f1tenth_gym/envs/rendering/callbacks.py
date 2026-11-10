"""Built-in render callbacks for common visualization tasks."""
from __future__ import annotations

from typing import TYPE_CHECKING
import numpy as np

if TYPE_CHECKING:
    from .renderer import EnvRenderer
    from ..lidar import LiDARConfig


__all__ = ["make_lidar_scan_callback"]


def polar_to_cartesian(
    ranges: np.ndarray,
    angles: np.ndarray,
    pose_x: float,
    pose_y: float,
    pose_theta: float,
    lidar_offset: tuple[float, float] = (0.0, 0.0),
) -> np.ndarray:
    """Convert polar LiDAR scan to Cartesian coordinates in world frame.
    
    Args:
        ranges: Array of range measurements (num_beams,).
        angles: Array of beam angles in radians (num_beams,).
        pose_x: Robot x position in world frame.
        pose_y: Robot y position in world frame.
        pose_theta: Robot heading in world frame (radians).
        lidar_offset: (x, y) offset of lidar from robot base_link.
        
    Returns:
        Array of shape (N, 2) with Cartesian (x, y) coordinates in world frame.
    """
    # Convert to Cartesian in lidar frame
    x_lidar = ranges * np.cos(angles)
    y_lidar = ranges * np.sin(angles)
    
    # Apply lidar offset in robot frame
    x_robot = x_lidar + lidar_offset[0]
    y_robot = y_lidar + lidar_offset[1]
    
    # Rotate to world frame
    cos_theta = np.cos(pose_theta)
    sin_theta = np.sin(pose_theta)
    
    x_world = pose_x + cos_theta * x_robot - sin_theta * y_robot
    y_world = pose_y + sin_theta * x_robot + cos_theta * y_robot
    
    return np.column_stack([x_world, y_world]).astype(np.float32)


def make_lidar_scan_callback(
    agent_id: str,
    lidar_config: "LiDARConfig",
    color: tuple[int, int, int] = (255, 0, 0),
    size: int = 3,
    subsample: int = 1,
):
    """Create a render callback that visualizes LiDAR scans as a point cloud.
    
    Usage:
        env = gym.make("f1tenth_gym:f1tenth-v0", config=cfg, render_mode="human")
        scan_callback = make_lidar_scan_callback("agent_0", cfg.lidar_config)
        env.unwrapped.add_render_callback(scan_callback)
    
    Args:
        agent_id: The agent ID whose scan to render (e.g., "agent_0").
        lidar_config: The LiDARConfig used by the environment.
        color: RGB tuple for point color (default: red).
        size: Point size in pixels (default: 3).
        subsample: Only render every Nth point for performance (default: 1).
        
    Returns:
        A callback function to pass to env.unwrapped.add_render_callback().
    """
    # Precompute angles
    angles = np.linspace(
        lidar_config.angle_min,
        lidar_config.angle_max,
        lidar_config.num_beams,
        dtype=np.float32,
    )
    lidar_offset = lidar_config.base_link_to_lidar_tf[:2]
    max_range = lidar_config.range_max
    
    # Apply subsampling to angles
    if subsample > 1:
        angles = angles[::subsample]
    
    # Renderer state (mutable closure)
    state = {"renderer": None}
    
    def callback(env_renderer: "EnvRenderer") -> None:
        obs = getattr(env_renderer, 'obs', None)
        if obs is None:
            return
        
        agent_obs = obs.get(agent_id)
        if agent_obs is None or "scan" not in agent_obs or "std_state" not in agent_obs:
            return
        
        scan = agent_obs["scan"]
        std_state = agent_obs["std_state"]
        
        # Subsample scan
        if subsample > 1:
            scan = scan[::subsample]
        
        # Filter invalid ranges
        valid = (scan > 0.1) & (scan <= max_range)
        scan_valid = scan[valid]
        angles_valid = angles[valid]
        
        if len(scan_valid) == 0:
            return
        
        # Convert to Cartesian
        points = polar_to_cartesian(
            scan_valid, angles_valid,
            float(std_state[0]), float(std_state[1]), float(std_state[4]),
            lidar_offset,
        )
        
        # Create or update renderer
        if state["renderer"] is None:
            state["renderer"] = env_renderer.get_points_renderer(points, color=color, size=size)
        else:
            state["renderer"].update(points)
    
    return callback

