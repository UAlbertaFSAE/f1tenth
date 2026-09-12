#!/usr/bin/env python3
"""Point the sim at one cone CSV.

Compute the ego spawn pose from it and wire that same CSV into cone_detector_sim
and map_generator's track_map_publisher.

Map is always maps/blank (a real empty occupancy grid) -- the track itself
is never rasterized into the map image, it's published separately as ground
truth by map_generator's track_map_publisher (/track_map) and driven live by
cone_detector_sim (/cone_positions, /cone_view_markers). set_track.py's only job
now is keeping the CSV path and spawn pose in sync across those two configs.

Usage: scripts/set_track.py <path/to/track.csv> [station_index]
"""

import csv
import math
import re
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
SIM_YAML = REPO_ROOT / "src/simulation/f1tenth_gym_ros/config/sim.yaml"
CONE_DETECTOR_CONFIG = REPO_ROOT / "src/simulation/cone_detector_sim/config/config.yaml"
TRACK_MAP_CONFIG = REPO_ROOT / "src/simulation/map_generator/config/config.yaml"


def station_center(stations, sid):
    """Midpoint of one cone gate, between its blue and yellow cone."""
    blue = stations[sid]["blue"]
    yellow = stations[sid]["yellow"]
    return ((blue[0] + yellow[0]) / 2, (blue[1] + yellow[1]) / 2)


def compute_spawn_pose(csv_path, station_index):
    """Spawn pose at a gate midpoint, facing the next gate."""
    stations = {}
    with open(csv_path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            sid = int(row["id"])
            stations.setdefault(sid, {})[row["color"]] = (
                float(row["x"]),
                float(row["y"]),
            )

    ids = sorted(stations.keys())
    if station_index not in stations or (station_index + 1) % len(ids) not in stations:
        raise ValueError(
            f"station {station_index} (or its successor) not found in {csv_path}"
        )

    c0 = station_center(stations, ids[station_index])
    c1 = station_center(stations, ids[(station_index + 1) % len(ids)])
    heading = math.atan2(c1[1] - c0[1], c1[0] - c0[0])
    return c0[0], c0[1], heading


def patch_sim_yaml(sx, sy, stheta):
    """Write the spawn pose into the gym bridge config, pinning the map to maps/blank."""
    text = SIM_YAML.read_text(encoding="utf-8")
    text = re.sub(r'map_path:\s*"[^"]*"', 'map_path: "maps/blank"', text, count=1)
    text = re.sub(
        r"^(\s*sx:)\s*[-0-9.]+", rf"\g<1> {sx:.4f}", text, count=1, flags=re.MULTILINE
    )
    text = re.sub(
        r"^(\s*sy:)\s*[-0-9.]+", rf"\g<1> {sy:.4f}", text, count=1, flags=re.MULTILINE
    )
    text = re.sub(
        r"^(\s*stheta:)\s*[-0-9.]+",
        rf"\g<1> {stheta:.4f}",
        text,
        count=1,
        flags=re.MULTILINE,
    )
    SIM_YAML.write_text(text, encoding="utf-8")


def patch_csv_path(config_path, csv_path):
    """Point one config's csv_path at the given track CSV."""
    text = config_path.read_text(encoding="utf-8")
    text = re.sub(
        r'^(\s*csv_path:)\s*".*"',
        rf'\g<1> "{csv_path}"',
        text,
        count=1,
        flags=re.MULTILINE,
    )
    config_path.write_text(text, encoding="utf-8")


def main():
    """Wire one track CSV into both sim configs and the gym spawn pose."""
    if len(sys.argv) not in (2, 3):
        print(
            f"Usage: {sys.argv[0]} <path/to/track.csv> [station_index]", file=sys.stderr
        )
        return 1

    csv_path = Path(sys.argv[1]).resolve()
    station_index = int(sys.argv[2]) if len(sys.argv) == 3 else 0

    if not csv_path.is_file():
        print(f"csv not found: {csv_path}", file=sys.stderr)
        return 1

    sx, sy, stheta = compute_spawn_pose(csv_path, station_index)

    patch_sim_yaml(sx, sy, stheta)
    patch_csv_path(CONE_DETECTOR_CONFIG, str(csv_path))
    patch_csv_path(TRACK_MAP_CONFIG, str(csv_path))

    print(
        f"csv={csv_path} map_path=maps/blank spawn=({sx:.3f}, {sy:.3f}, {stheta:.4f})"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
