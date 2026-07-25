"""CSV export/import: id,x,y,color"""
import csv
import numpy as np


def export_csv(path, cone_pairs):
    """Write id,x,y,color. No axis transform: editor (x,y) in meters is
    written straight through, so whatever is drawn (position AND direction)
    reproduces identically in ROS/RViz's map frame.
    """
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "x", "y", "color"])
        for pair in cone_pairs:
            bx, by = pair["blue"]
            yx, yy = pair["yellow"]
            writer.writerow([pair["id"], bx, by, "blue"])
            writer.writerow([pair["id"], yx, yy, "yellow"])


def import_csv(path):
    """Returns dict: {pair_id: {"blue": (x,y) or None, "yellow": (x,y) or None}}
    preserving first-seen id order.
    """
    pairs = {}
    order = []
    with open(path, "r", newline="") as f:
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


def reconstruct_centerline(ordered_pairs):
    centers = []
    for pair in ordered_pairs:
        if pair["blue"] is not None and pair["yellow"] is not None:
            b = np.array(pair["blue"])
            y = np.array(pair["yellow"])
            centers.append(tuple((b + y) / 2.0))
    return centers
