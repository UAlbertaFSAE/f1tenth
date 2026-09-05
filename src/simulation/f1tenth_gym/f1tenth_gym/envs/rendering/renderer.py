from __future__ import annotations

from abc import abstractmethod, ABC
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Union

import numpy as np

if TYPE_CHECKING:
    pass

@dataclass
class RenderSpec:
    window_size: int = 800
    focus_on: str | None = "agent_0"
    zoom_in_factor: float = 0.5
    bigger_car_when_map_centered: bool = True
    render_map_img: bool = True
    show_wheels: bool = True
    car_tickness: int = 1
    show_info: bool = True
    vehicle_palette: tuple[str, ...] = (
        "#FD3754",
        "#377eb8",
        "#984ea3",
        "#e41a1c",
        "#ff7f00",
        "#a65628",
        "#f781bf",
        "#888888",
        "#a6cee3",
        "#b2df8a",
    )
    render_type: str = "pyqt6gl"
    car_model: str = "2d"
    frame_output_method: str = "offscreen"
    frame_output_info_label: bool = True


class EnvRenderer(ABC):
    """
    Abstract class for rendering the environment.
    """
    @abstractmethod
    def add_renderer_callback(self, callback_fn):
        """
        Add a custom callback for visualization.

        Parameters
        ----------
        callback_fn : Callable[[EnvRenderer], None]
            callback function to be called at every rendering step
        """
        raise NotImplementedError()
    
    @abstractmethod
    def update(self, obs: dict) -> None:
        """
        Update the state to be rendered.
        This is called at every rendering call.

        Parameters
        ----------
        obs : dict
            observations from the env to be rendered
        """
        raise NotImplementedError()    

    @abstractmethod
    def render(self):
        """
        Render the current state in a frame.
        """
        raise NotImplementedError()

    @abstractmethod
    def close(self):
        """
        Close the rendering window.
        """
        raise NotImplementedError()

    @abstractmethod
    def get_points_renderer(
        self,
        points: Union[list, np.ndarray],
        color: Optional[tuple[int, int, int]] = (0, 0, 255),
        size: Optional[int] = 1,
        **kwargs,
    ) -> "ObjectRenderer":
        """
        Get a point renderer for visualizing points on the map.

        Parameters
        ----------
        points : Union[list, np.ndarray]
            Array of shape (N, 2) or (N, 3) with point coordinates.
        color : tuple[int, int, int], optional
            RGB color tuple, by default (0, 0, 255) (blue).
        size : int, optional
            Size of points in pixels, by default 1.

        Returns
        -------
        ObjectRenderer
            A renderer object that can be updated with new points.
        """
        raise NotImplementedError()

    @abstractmethod
    def get_lines_renderer(
        self,
        points: Union[list, np.ndarray],
        color: Optional[tuple[int, int, int]] = (0, 0, 255),
        size: Optional[int] = 1,
        **kwargs,
    ) -> "ObjectRenderer":
        """
        Get a line renderer for visualizing connected line segments.

        Parameters
        ----------
        points : Union[list, np.ndarray]
            Array of shape (N, 2) or (N, 3) with point coordinates forming line segments.
        color : tuple[int, int, int], optional
            RGB color tuple, by default (0, 0, 255) (blue).
        size : int, optional
            Line width in pixels, by default 1.

        Returns
        -------
        ObjectRenderer
            A renderer object that can be updated with new points.
        """
        raise NotImplementedError()

    @abstractmethod
    def get_closed_lines_renderer(
        self,
        points: Union[list, np.ndarray],
        color: Optional[tuple[int, int, int]] = (0, 0, 255),
        size: Optional[int] = 1,
        **kwargs,
    ) -> "ObjectRenderer":
        """
        Get a closed line renderer for visualizing polygons or closed paths.

        Parameters
        ----------
        points : Union[list, np.ndarray]
            Array of shape (N, 2) or (N, 3) with point coordinates forming a closed shape.
        color : tuple[int, int, int], optional
            RGB color tuple, by default (0, 0, 255) (blue).
        size : int, optional
            Line width in pixels, by default 1.

        Returns
        -------
        ObjectRenderer
            A renderer object that can be updated with new points.
        """
        raise NotImplementedError()


class ObjectRenderer(ABC):
    
    @abstractmethod
    def __init__(self):
        """
        Initialize the point renderer.
        This should set up the necessary parameters for rendering points.
        """
        pass
    
    @abstractmethod
    def update(self, points: np.ndarray) -> None:
        """
        Update the renderer with new point data.
        
        Parameters
        ----------
        points : np.ndarray
            Array of shape (N, 2) or (N, 3) with point coordinates.
        """
        raise NotImplementedError()