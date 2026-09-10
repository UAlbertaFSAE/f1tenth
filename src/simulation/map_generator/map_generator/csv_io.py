"""CSV export/import: id,x,y,color."""

import csv
from collections.abc import Sequence

import numpy as np


def export_csv(path: str, cone_pairs: Sequence[dict]) -> None:
    """Write id,x,y,color.

    No axis transform: editor (x,y) in meters is written straight through, so
    whatever is drawn (position AND direction) reproduces identically in
    ROS/RViz's map frame.
    """
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "x", "y", "color"])
        for pair in cone_pairs:
            bx, by = pair["blue"]
            yx, yy = pair["yellow"]
            writer.writerow([pair["id"], bx, by, "blue"])
            writer.writerow([pair["id"], yx, yy, "yellow"])


def import_csv(path: str) -> list:
    """Read a cone CSV into pairs, preserving first-seen id order.

    Returns dict: {pair_id: {"blue": (x,y) or None, "yellow": (x,y) or None}}.
    """
    pairs: dict[int, dict[str, tuple[float, float] | None]] = {}
    order: list[int] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pid = int(row["id"])
            x, y = float(row["x"]), float(row["y"])
            color = row["color"].strip().lower()
            if pid not in pairs:
                pairs[pid] = {"blue": None, "yellow": None}
                order.append(pid)
            if color in ("blue", "yellow"):
                pairs[pid][color] = (x, y)

    ordered_pairs = [{"id": pid, **pairs[pid]} for pid in order]
    return ordered_pairs


def reconstruct_centerline(ordered_pairs: Sequence[dict]) -> np.ndarray:
    """Rebuild the centerline as the midpoint of each complete cone gate."""
    centers: list[tuple[float, float]] = []
    for pair in ordered_pairs:
        if pair["blue"] is not None and pair["yellow"] is not None:
            b = np.array(pair["blue"])
            y = np.array(pair["yellow"])
            mid = (b + y) / 2.0
            centers.append((float(mid[0]), float(mid[1])))
    line: np.ndarray = np.array(centers, dtype=float).reshape(-1, 2)
    return line
