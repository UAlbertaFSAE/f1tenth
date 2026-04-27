from __future__ import annotations

from dataclasses import dataclass, replace

__all__ = ["LiDARConfig"]


@dataclass(frozen=True)
class LiDARConfig:
    """Configuration for the simulated LiDAR sensor.

    Attributes:
        enabled: Whether LiDAR scanning is enabled.
        num_beams: Number of laser beams in the scan.
        field_of_view: Total angular field of view in radians.
        angle_min: Start angle of scan in radians (default: -field_of_view/2).
        angle_max: End angle of scan in radians (default: +field_of_view/2).
        range_min: Minimum range in meters, readings below are clipped.
        range_max: Maximum range in meters, readings above are clipped.
        noise_std: Standard deviation of Gaussian noise on range readings.
        base_link_to_lidar_tf: (x, y, yaw) offset from base_link in meters/radians.
    """

    enabled: bool = True
    num_beams: int = 1080
    field_of_view: float = 4.712389
    angle_min: float | None = None
    angle_max: float | None = None
    range_min: float = 0.0
    range_max: float = 30.0
    noise_std: float = 0.01
    # (x, y, yaw) offset from base_link in meters/radians.
    base_link_to_lidar_tf: tuple[float, float, float] = (0.275, 0.0, 0.0)

    def __post_init__(self) -> None:
        # Set angle_min/angle_max defaults from field_of_view if not specified
        if self.angle_min is None:
            object.__setattr__(self, "angle_min", -self.field_of_view / 2.0)
        if self.angle_max is None:
            object.__setattr__(self, "angle_max", self.field_of_view / 2.0)

        # Validation
        if self.num_beams < 1:
            raise ValueError(f"num_beams must be >= 1, got {self.num_beams}")
        if self.field_of_view <= 0:
            raise ValueError(f"field_of_view must be > 0, got {self.field_of_view}")
        if self.range_max <= 0:
            raise ValueError(f"range_max must be > 0, got {self.range_max}")
        if self.range_min < 0:
            raise ValueError(f"range_min must be >= 0, got {self.range_min}")
        if self.range_min >= self.range_max:
            raise ValueError(
                f"range_min ({self.range_min}) must be less than range_max ({self.range_max})"
            )
        if self.noise_std < 0:
            raise ValueError(f"noise_std must be >= 0, got {self.noise_std}")
        if self.angle_min >= self.angle_max:
            raise ValueError(
                f"angle_min ({self.angle_min}) must be less than angle_max ({self.angle_max})"
            )
        import math
        if self.angle_min < -math.pi:
            raise ValueError(
                f"angle_min ({self.angle_min}) must be >= -π (-180°). "
                f"Did you pass degrees instead of radians? Use np.deg2rad() to convert."
            )
        if self.angle_max > math.pi:
            raise ValueError(
                f"angle_max ({self.angle_max}) must be <= π (180°). "
                f"Did you pass degrees instead of radians? Use np.deg2rad() to convert."
            )

    @property
    def angle_increment(self) -> float:
        """Angular distance between consecutive measurements in radians."""
        if self.num_beams <= 1:
            return 0.0
        return (self.angle_max - self.angle_min) / (self.num_beams - 1)

    @property
    def maximum_range(self) -> float:
        """Alias for range_max for backwards compatibility."""
        return self.range_max

    def with_updates(self, **changes: object) -> "LiDARConfig":
        return replace(self, **changes)
