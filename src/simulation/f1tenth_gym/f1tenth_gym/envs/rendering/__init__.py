import os
from typing import Optional, TYPE_CHECKING

from .renderer import RenderSpec, EnvRenderer, ObjectRenderer
from .callbacks import make_lidar_scan_callback
from ..dynamic_models import VehicleParameters

if TYPE_CHECKING:
    from ..track import Track

__all__ = [
    "RenderSpec",
    "EnvRenderer",
    "ObjectRenderer",
    "make_lidar_scan_callback",
    "make_renderer",
]

def make_renderer(
    params: VehicleParameters,
    track: "Track",
    agent_ids: list[str],
    render_mode: Optional[str] = None,
    render_fps: Optional[int] = 100,
    render_spec: RenderSpec = RenderSpec(),
) -> tuple[EnvRenderer, RenderSpec]:
    """Return an instance of the renderer and the rendering specification."""
    render_spec = render_spec

    if render_spec.render_type == "pyqt6":
        if render_mode in ["rgb_array", "rgb_array_list"]:
            os.environ["QT_QPA_PLATFORM"] = "offscreen"
        from .rendering_pyqt import PyQtEnvRenderer as EnvRenderer
    elif render_spec.render_type == "pyqt6gl":
        if render_mode in ["rgb_array", "rgb_array_list"]:
            os.environ["QT_QPA_PLATFORM"] = "offscreen"
            from .rendering_pyqt import PyQtEnvRenderer as EnvRenderer
        else:
            from .rendering_pyqtgl import PyQtEnvRendererGL as EnvRenderer
    else:
        raise ValueError(f"Unknown render type: {render_spec.render_type}")

    if render_mode in ["human", "rgb_array", "unlimited", "human_fast"]:
        renderer = EnvRenderer(
            params=params,
            track=track,
            agent_ids=agent_ids,
            render_spec=render_spec,
            render_mode=render_mode,
            render_fps=render_fps,
        )
    else:
        renderer = None
    return renderer, render_spec
