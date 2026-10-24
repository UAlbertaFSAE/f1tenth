#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

CONE_RADIUS = 0.12
CONE_HEIGHT = 0.30


def cone_link(name: str, x: float, y: float, color: str) -> str:
    z = CONE_HEIGHT / 2.0

    if color == "blue":
        rgba = "0 0 1 1"
    elif color == "yellow":
        rgba = "1 1 0 1"
    else:
        rgba = "1 0.5 0 1"

    return f"""
    <link name="{name}">
      <pose>{x} {y} {z} 0 0 0</pose>

      <visual name="visual">
        <geometry>
          <cone>
            <radius>{CONE_RADIUS}</radius>
            <length>{CONE_HEIGHT}</length>
          </cone>
        </geometry>

        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
        </material>
      </visual>

      <collision name="collision">
        <geometry>
          <cone>
            <radius>{CONE_RADIUS}</radius>
            <length>{CONE_HEIGHT}</length>
          </cone>
        </geometry>
      </collision>
    </link>
"""


def generate(csv_path: Path, output_path: Path) -> None:
    links = []

    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)

        for row_index, row in enumerate(reader):
            cone_id = int(row["id"])
            x = float(row["x"])
            y = float(row["y"])
            color = row["color"].strip().lower()

            name = f"{color}_cone_{cone_id}_{row_index}"

            links.append(
                cone_link(
                    name,
                    x,
                    y,
                    color,
                )
            )

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="generated_track">
    <static>true</static>
{"".join(links)}
  </model>
</sdf>
"""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(sdf)

    print(f"Generated {len(links)} cones")
    print(f"Wrote: {output_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: generate_track.py <cones.csv> <output_model.sdf>")
        sys.exit(1)

    generate(
        Path(sys.argv[1]),
        Path(sys.argv[2]),
    )
