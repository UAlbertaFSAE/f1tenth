#!/usr/bin/env python3
"""Sync track_type, map_path, and ego spawn pose for one named track.

detection_generator's track_type and f1tenth_gym_ros's map_path/spawn pose
are independent parameters in two separate config files with no automatic
coupling -- get them out of sync (e.g. track_type: levine but map_path still
pointing at "maps/straight") and cones/map/car all disagree with each other.
This script is the single place that knows the correct pairing for each
track, computed the same way they were worked out by hand: the map file that
matches the track's cone-CSV coordinate frame, and a spawn pose taken
directly from that CSV (so the car starts already aligned with the track).

Usage: scripts/set_track.py <straight|eight|curved|levine>
"""
import csv
import math
import re
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
SIM_YAML = REPO_ROOT / "src/simulation/f1tenth_gym_ros/config/sim.yaml"
DETECTION_CONFIG = REPO_ROOT / "src/simulation/detection_generator/config/config.yaml"
DATA_DIR = REPO_ROOT / "src/simulation/detection_generator/data"

# track_type -> (map name under f1tenth_gym_ros/config/maps, cone CSV,
# station index to spawn at). Station 0 works for the synthetic tracks
# (straight/eight/curved all start at the origin heading 0); levine's
# station 0 sits right at a corner/narrow join near the loop's leftmost
# edge (the contour-tracing start point) and caused an immediate collision,
# so it spawns at station 6 instead (mid-straightaway).
TRACKS = {
    "straight": ("straight", "straight.csv", 0),
    "eight": ("eight", "eight.csv", 0),
    "curved": ("blank", "curved.csv", 0),
    "levine": ("levine", "levine.csv", 6),
}


def station_center(stations, sid):
    blue = stations[sid]["blue"]
    yellow = stations[sid]["yellow"]
    return ((blue[0] + yellow[0]) / 2, (blue[1] + yellow[1]) / 2)


def compute_spawn_pose(csv_path, station_index):
    stations = {}
    with open(csv_path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            sid = int(row["id"])
            stations.setdefault(sid, {})[row["color"]] = (float(row["x"]), float(row["y"]))

    ids = sorted(stations.keys())
    if station_index not in stations or (station_index + 1) % len(ids) not in stations:
        raise ValueError(f"station {station_index} (or its successor) not found in {csv_path}")

    c0 = station_center(stations, ids[station_index])
    c1 = station_center(stations, ids[(station_index + 1) % len(ids)])
    heading = math.atan2(c1[1] - c0[1], c1[0] - c0[0])
    return c0[0], c0[1], heading


def patch_sim_yaml(map_name, sx, sy, stheta):
    text = SIM_YAML.read_text(encoding="utf-8")
    text = re.sub(r'map_path:\s*"[^"]*"', f'map_path: "maps/{map_name}"', text, count=1)
    text = re.sub(r"^(\s*sx:)\s*[-0-9.]+", rf"\g<1> {sx:.4f}", text, count=1, flags=re.MULTILINE)
    text = re.sub(r"^(\s*sy:)\s*[-0-9.]+", rf"\g<1> {sy:.4f}", text, count=1, flags=re.MULTILINE)
    text = re.sub(
        r"^(\s*stheta:)\s*[-0-9.]+", rf"\g<1> {stheta:.4f}", text, count=1, flags=re.MULTILINE
    )
    SIM_YAML.write_text(text, encoding="utf-8")


def patch_detection_config(track_type):
    text = DETECTION_CONFIG.read_text(encoding="utf-8")
    text = re.sub(r"^(\s*track_type:)\s*\S+", rf"\g<1> {track_type}", text, count=1, flags=re.MULTILINE)
    DETECTION_CONFIG.write_text(text, encoding="utf-8")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in TRACKS:
        print(f"Usage: {sys.argv[0]} <{'|'.join(TRACKS)}>", file=sys.stderr)
        return 1

    track_type = sys.argv[1]
    map_name, csv_name, station_index = TRACKS[track_type]
    sx, sy, stheta = compute_spawn_pose(DATA_DIR / csv_name, station_index)

    patch_sim_yaml(map_name, sx, sy, stheta)
    patch_detection_config(track_type)

    print(f"track_type={track_type} map_path=maps/{map_name} "
          f"spawn=({sx:.3f}, {sy:.3f}, {stheta:.4f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
