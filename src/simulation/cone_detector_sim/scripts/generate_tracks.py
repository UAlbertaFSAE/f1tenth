#!/usr/bin/env python3
"""Generate straight/eight/curved track cone station CSVs for cone_detector_sim.

Builds each track's centerline out of line/arc segments and places cone
stations along it at curvature-adaptive arc-length spacing. The CSV is just
the ordered station list (id = station index along the track, increasing
with arc length) - the cone_detector_sim node computes which stations are
currently visible live, from the car's actual odometry, so no per-frame
simulation or duplication needs to happen here.
"""

import csv
import math
from dataclasses import dataclass
from pathlib import Path

# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------

BASE_SPACING = 1.0      # cone-pair spacing on straights (m)
MIN_SPACING = 0.4       # tightest spacing allowed on sharp curves (m)
MAX_SPACING = 5.0       # hard cap on cone-pair spacing (m)
CURVATURE_SPACING_GAIN = 4.5

HALF_WIDTH = 1.5        # fixed -> 3.0 m track width everywhere in the simulator

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


# ---------------------------------------------------------------------------
# Path segments (line / arc), each parameterized by local arc length
# ---------------------------------------------------------------------------

@dataclass
class Line:
    start: tuple
    heading: float
    length: float

    def sample(self, s: float):
        x = self.start[0] + s * math.cos(self.heading)
        y = self.start[1] + s * math.sin(self.heading)
        return x, y, self.heading, 0.0

    def end_point(self):
        return self.sample(self.length)[:2]

    def end_heading(self):
        return self.heading


@dataclass
class Arc:
    center: tuple
    radius: float
    start_angle: float
    delta_angle: float  # signed; total turn = |delta_angle|, sign = turn direction (+ccw/left)

    @property
    def length(self):
        return abs(self.delta_angle) * self.radius

    def sample(self, s: float):
        direction = 1.0 if self.delta_angle >= 0 else -1.0
        theta = self.start_angle + direction * (s / self.radius)
        x = self.center[0] + self.radius * math.cos(theta)
        y = self.center[1] + self.radius * math.sin(theta)
        heading = theta + direction * (math.pi / 2.0)
        curvature = direction / self.radius
        return x, y, heading, curvature

    def end_point(self):
        return self.sample(self.length)[:2]

    def end_heading(self):
        return self.sample(self.length)[2]


class Path:
    """A sequence of segments sampled by cumulative arc length."""

    def __init__(self, segments):
        self.segments = segments
        self.offsets = []
        total = 0.0
        for seg in segments:
            self.offsets.append(total)
            total += seg.length
        self.total_length = total

    def sample(self, s: float):
        s = max(0.0, min(s, self.total_length - 1e-9))
        idx = 0
        for i in range(len(self.segments) - 1, -1, -1):
            if s >= self.offsets[i]:
                idx = i
                break
        return self.segments[idx].sample(s - self.offsets[idx])


def build_straight_path() -> Path:
    return Path([Line(start=(0.0, 0.0), heading=0.0, length=60.0)])


def build_eight_path() -> Path:
    lead_in = 10.0
    radius = 8.0
    start = (lead_in, 0.0)
    return Path([
        Line(start=(0.0, 0.0), heading=0.0, length=lead_in),
        # left loop: full revolution back to the crossing point, turning left (+curvature)
        Arc(center=(start[0], radius), radius=radius,
            start_angle=-math.pi / 2.0, delta_angle=2.0 * math.pi),
        # right loop: full revolution back to the crossing point, turning right (-curvature)
        Arc(center=(start[0], -radius), radius=radius,
            start_angle=math.pi / 2.0, delta_angle=-2.0 * math.pi),
    ])


def build_curved_path() -> Path:
    """Closed rounded-rectangle loop: alternating relaxed/sharp 90 deg corners."""
    straight_a = 25.0   # top/bottom straight length
    straight_b = 15.0   # left/right straight length
    relaxed_r = 9.0     # relaxed corner radius (opposite corners)
    sharp_r = 4.5       # sharp corner radius (opposite corners)
    quarter = math.pi / 2.0

    segments = []
    pos = (0.0, 0.0)
    heading = 0.0

    def add_line(length):
        nonlocal pos, heading
        seg = Line(start=pos, heading=heading, length=length)
        segments.append(seg)
        pos = seg.end_point()

    def add_corner(radius):
        nonlocal pos, heading
        # left vector relative to current heading
        left = (-math.sin(heading), math.cos(heading))
        center = (pos[0] + radius * left[0], pos[1] + radius * left[1])
        start_angle = math.atan2(pos[1] - center[1], pos[0] - center[0])
        seg = Arc(center=center, radius=radius,
                  start_angle=start_angle, delta_angle=quarter)
        segments.append(seg)
        pos = seg.end_point()
        heading = seg.end_heading()

    add_line(straight_a)
    add_corner(relaxed_r)
    add_line(straight_b)
    add_corner(sharp_r)
    add_line(straight_a)
    add_corner(relaxed_r)
    add_line(straight_b)
    add_corner(sharp_r)

    return Path(segments)


TRACKS = {
    "straight": (build_straight_path, False),
    "eight": (build_eight_path, False),
    "curved": (build_curved_path, True),
}


# ---------------------------------------------------------------------------
# Cone station placement (curvature-adaptive arc-length sampling)
# ---------------------------------------------------------------------------

@dataclass
class Station:
    s: float
    x: float
    y: float
    heading: float
    half_width: float
    blue: tuple
    yellow: tuple


def spacing_for_curvature(curvature: float) -> float:
    spacing = BASE_SPACING / (1.0 + CURVATURE_SPACING_GAIN * abs(curvature))
    return max(MIN_SPACING, min(MAX_SPACING, spacing))


def make_station(path: Path, s: float) -> Station:
    x, y, heading, _curvature = path.sample(s)
    left = (x - HALF_WIDTH * math.sin(heading), y + HALF_WIDTH * math.cos(heading))
    right = (x + HALF_WIDTH * math.sin(heading), y - HALF_WIDTH * math.cos(heading))
    return Station(s=s, x=x, y=y, heading=heading, half_width=HALF_WIDTH,
                   blue=left, yellow=right)


def build_stations(path: Path, closed: bool) -> list:
    stations = []
    s = 0.0
    while s < path.total_length - 1e-9:
        stations.append(make_station(path, s))
        _, _, _, curvature = path.sample(s)
        s += spacing_for_curvature(curvature)

    if closed:
        # keep the loop-closing gap within the max spacing cap
        gap = path.total_length - stations[-1].s
        if gap > MAX_SPACING:
            stations.append(make_station(path, stations[-1].s + gap / 2.0))

    return stations


# ---------------------------------------------------------------------------
# CSV output: one station per id, in arc-length order
# ---------------------------------------------------------------------------

def write_csv(path: Path, stations: list) -> None:
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["id", "x", "y", "color"])
        for station_id, station in enumerate(stations):
            writer.writerow([station_id, round(station.blue[0], 3), round(station.blue[1], 3), "blue"])
            writer.writerow([station_id, round(station.yellow[0], 3), round(station.yellow[1], 3), "yellow"])


def main() -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    for name, (builder, closed) in TRACKS.items():
        path = builder()
        stations = build_stations(path, closed=closed)
        out_path = DATA_DIR / f"{name}.csv"
        write_csv(out_path, stations)
        print(f"{name}: {len(stations)} stations ({2 * len(stations)} cone rows), "
              f"track length {path.total_length:.1f} m -> {out_path}")


if __name__ == "__main__":
    main()
