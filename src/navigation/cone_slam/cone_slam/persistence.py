"""Strict versioned map persistence with atomic writes and immutable references."""

import json
import os
import tempfile
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from .geometry import covariance
from .mapper import Landmark, Mapper, Settings

SCHEMA = 1
MAX_BYTES = 32 * 1024 * 1024
FRAMES = {
    "map": "map",
    "odom": "odom",
    "body": "base_link",
    "units": "metres_radians",
    "landmark": "cone_axis_ground_xy",
}


def _json(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json(v) for v in value]
    return value


def save(mapper: Mapper, filename: str, source_commit: str = "unknown") -> None:
    """Save finite landmarks and source graph evidence without any driving path."""
    if mapper.mode == "localization":
        raise ValueError("localization_reference_is_read_only")
    data = {
        "schema": SCHEMA,
        "map_id": mapper.map_id,
        "revision": mapper.revision,
        "next_id": mapper.next_id,
        "frames": FRAMES,
        "provenance": {
            "source_commit": source_commit,
            "settings": asdict(mapper.settings),
            "calibration_id": mapper.settings.calibration_id,
        },
        "landmarks": [
            dict(
                asdict(value),
                position=mapper.graph.points[key],
                covariance=mapper.graph.point_cov[key],
            )
            for key, value in mapper.landmarks.items()
            if value.state in ("confirmed", "inactive")
        ],
        "graph": {
            "keyframes": [asdict(k) for k in mapper.graph.keys],
            "factors": [asdict(f) for f in mapper.graph.factors],
            "odometry_edges": mapper.graph.edges,
        },
    }
    encoded = json.dumps(
        _json(data), allow_nan=False, sort_keys=True, separators=(",", ":")
    ).encode()
    if len(encoded) > MAX_BYTES:
        raise ValueError("map_file_capacity")
    path = Path(filename).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=path.parent, prefix=path.name + ".", delete=False
        ) as handle:
            temporary = handle.name
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


def load(filename: str, settings: Settings, mode: str) -> Mapper:
    """Load a saved reference without reusing its historical odom origin."""
    if mode not in ("localization", "update"):
        raise ValueError("loading_requires_localization_or_update")
    path = Path(filename).expanduser()
    if path.stat().st_size > MAX_BYTES:
        raise ValueError("map_file_capacity")

    def reject_constant(value: str) -> None:
        raise ValueError(f"nonfinite map value: {value}")

    data = json.loads(path.read_text(), parse_constant=reject_constant)
    if data.get("schema") != SCHEMA or data.get("frames") != FRAMES:
        raise ValueError("incompatible_schema_or_frames")
    if data.get("provenance", {}).get("calibration_id") != settings.calibration_id:
        raise ValueError("incompatible_calibration")
    if not isinstance(data.get("map_id"), str) or not data["map_id"]:
        raise ValueError("invalid_map_id")
    if not isinstance(data.get("revision"), int) or data["revision"] < 0:
        raise ValueError("invalid_revision")
    rows = data.get("landmarks")
    if not isinstance(rows, list) or not 4 <= len(rows) <= settings.max_landmarks:
        raise ValueError("insufficient_or_excessive_landmarks")
    mapper = Mapper(settings, mode)
    mapper.map_id, mapper.revision = data["map_id"], data["revision"]
    for row in rows:
        identity = row["id"]
        position = np.array(row["position"], dtype=float)
        colors = np.array(row["colors"], dtype=float)
        if (
            not isinstance(identity, int)
            or not 0 < identity < 2**64
            or identity in mapper.landmarks
        ):
            raise ValueError("invalid_landmark_id")
        if position.shape != (2,) or not np.isfinite(position).all():
            raise ValueError("invalid_landmark_position")
        if colors.shape != (4,) or not np.isfinite(colors).all() or np.any(colors < 0):
            raise ValueError("invalid_color_evidence")
        if (
            row["state"] not in ("confirmed", "inactive")
            or not isinstance(row["hits"], int)
            or row["hits"] < 1
        ):
            raise ValueError("invalid_landmark_state")
        mapper.landmarks[identity] = Landmark(
            identity,
            state=row["state"],
            hits=row["hits"],
            colors=colors,
            last_seen=-1e9,
        )
        mapper.graph.points[identity] = position
        mapper.graph.point_cov[identity] = covariance(
            np.array(row["covariance"]), 2, floor=0.0
        )
    if (
        not isinstance(data.get("next_id"), int)
        or data["next_id"] <= max(mapper.landmarks)
        or data["next_id"] >= 2**64
    ):
        raise ValueError("invalid_next_id")
    mapper.next_id = data["next_id"]
    mapper.reference_ids = set(mapper.landmarks)
    mapper.graph.fixed = set(mapper.landmarks)
    mapper.reason = "waiting_for_relocalization"
    return mapper
