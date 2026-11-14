"""Warm-started sparse robust pose/landmark graph with a fixed gauge."""

from dataclasses import dataclass, field
from time import monotonic
from typing import Any, cast

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import splu

from .geometry import between, covariance, inverse, transform, wrap


@dataclass
class Keyframe:
    """Original odometry and optimized pose at an immutable capture time."""

    stamp: float
    odom: np.ndarray
    pose: np.ndarray
    cov: np.ndarray


@dataclass
class Factor:
    """Original body-relative measurement and capture-pose offset."""

    key: int
    landmark: int
    measurement: np.ndarray
    offset: np.ndarray
    cov: np.ndarray
    evidence: str
    sensor: str
    calibration: str
    provenance: dict[str, Any] = field(default_factory=dict)


class Graph:
    """Retain the complete bounded graph; stop growing rather than discard history."""

    def __init__(self) -> None:
        """Initialize an empty graph without inventing a prior map."""
        self.keys: list[Keyframe] = []
        self.points: dict[int, np.ndarray] = {}
        self.fixed: set[int] = set()
        self.factors: list[Factor] = []
        self.edges: list[tuple[int, int, np.ndarray, np.ndarray]] = []
        self.pose_cov = np.diag([0.25, 0.25, 0.04])
        self.point_cov: dict[int, np.ndarray] = {}
        self.optimization_ms = 0.0
        self.last_error = "not_optimized"

    def optimize(self, budget_s: float = 0.25, iterations: int = 12) -> bool:
        """Optimize a sparse graph transactionally, rejecting overdue/nonfinite solves."""
        start = monotonic()
        if not self.keys or not self.factors:
            return False
        # Mapping fixes the initial pose exactly. Localization fixes the reference
        # landmarks instead, allowing every pose to move relative to the saved map.
        free_pose = list(range(0 if self.fixed else 1, len(self.keys)))
        free_point = sorted(set(self.points) - self.fixed)
        pose_col = {key: 3 * i for i, key in enumerate(free_pose)}
        point_col = {
            key: 3 * len(free_pose) + 2 * i for i, key in enumerate(free_point)
        }
        values = [self.keys[k].pose for k in free_pose] + [
            self.points[k] for k in free_point
        ]
        if not values:
            return False
        initial = np.concatenate(values)
        rows = 3 * len(self.edges) + 2 * len(self.factors)
        sparsity = lil_matrix((rows, len(initial)), dtype=int)
        for i, (a, b, _, _) in enumerate(self.edges):
            for k in (a, b):
                if k in pose_col:
                    sparsity[3 * i : 3 * i + 3, pose_col[k] : pose_col[k] + 3] = 1
        for i, factor in enumerate(self.factors):
            row = 3 * len(self.edges) + 2 * i
            if factor.key in pose_col:
                col = pose_col[factor.key]
                sparsity[row : row + 2, col : col + 3] = 1
            if factor.landmark in point_col:
                col = point_col[factor.landmark]
                sparsity[row : row + 2, col : col + 2] = 1
        edge_weights = [np.linalg.inv(np.linalg.cholesky(e[3])) for e in self.edges]
        weights = [np.linalg.inv(np.linalg.cholesky(f.cov)) for f in self.factors]
        edge_a = np.array([e[0] for e in self.edges], dtype=int)
        edge_b = np.array([e[1] for e in self.edges], dtype=int)
        observed_keys = np.array([f.key for f in self.factors], dtype=int)
        point_ids = sorted(self.points)
        point_index = {key: i for i, key in enumerate(point_ids)}
        observed_points = np.array([point_index[f.landmark] for f in self.factors])
        # Offset is from keyframe body to body at observation capture time. Its
        # uncertainty is in the measurement factor; keyframe pose is shared.
        measurements = np.array(
            [transform(f.offset, f.measurement) for f in self.factors]
        )
        base_poses = np.array([key.pose for key in self.keys])
        base_points = np.array([self.points[k] for k in point_ids])

        def unpack(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            poses, points = base_poses.copy(), base_points.copy()
            for k, col in pose_col.items():
                poses[k] = x[col : col + 3]
            for k, col in point_col.items():
                points[point_index[k]] = x[col : col + 2]
            return poses, points

        def residual(x: np.ndarray) -> np.ndarray:
            if monotonic() - start > budget_s:
                raise TimeoutError("optimization_budget")
            poses, points = unpack(x)
            results = []
            if len(self.edges):
                delta = poses[edge_b] - poses[edge_a]
                c, s = np.cos(poses[edge_a, 2]), np.sin(poses[edge_a, 2])
                prediction = np.column_stack(
                    (
                        c * delta[:, 0] + s * delta[:, 1],
                        -s * delta[:, 0] + c * delta[:, 1],
                        delta[:, 2],
                    )
                )
                error = prediction - np.array([e[2] for e in self.edges])
                error[:, 2] = np.arctan2(np.sin(error[:, 2]), np.cos(error[:, 2]))
                results.append(np.einsum("nij,nj->ni", edge_weights, error).ravel())
            delta = points[observed_points] - poses[observed_keys, :2]
            c, s = np.cos(poses[observed_keys, 2]), np.sin(poses[observed_keys, 2])
            prediction = np.column_stack(
                (c * delta[:, 0] + s * delta[:, 1], -s * delta[:, 0] + c * delta[:, 1])
            )
            results.append(
                np.einsum("nij,nj->ni", weights, prediction - measurements).ravel()
            )
            return cast(np.ndarray, np.concatenate(results))

        try:
            result = least_squares(
                residual,
                initial,
                jac_sparsity=sparsity.tocsr(),
                loss="huber",
                f_scale=2.0,
                max_nfev=iterations,
                tr_solver="lsmr",
                ftol=1e-4,
                xtol=1e-4,
                gtol=1e-4,
            )
            if (
                not np.isfinite(result.x).all()
                or result.cost > 0.5 * np.sum(residual(initial) ** 2) + 1e-8
            ):
                raise ValueError("invalid_optimization")
            # Sparse marginal solves include pose-landmark correlations. Do not
            # report the inverse diagonal of J'J as marginal covariance.
            information = (result.jac.T @ result.jac).tocsc()
            lu = splu(information)
            marginals: dict[int, np.ndarray] = {}

            def marginal(col: int, size: int) -> np.ndarray:
                rhs = np.zeros((len(initial), size))
                rhs[col : col + size] = np.eye(size)
                block = lu.solve(rhs)[col : col + size]
                return covariance((block + block.T) / 2, size)

            pose_cov = (
                np.eye(3) * 1e-6
                if len(self.keys) == 1 and not self.fixed
                else self.pose_cov
            )
            if len(self.keys) - 1 in pose_col:
                pose_cov = marginal(pose_col[len(self.keys) - 1], 3)
            # Refresh recent landmarks first; old covariances remain conservative
            # initialization values until visited, rather than fake zero noise.
            recent = list(dict.fromkeys(f.landmark for f in reversed(self.factors)))[
                :32
            ]
            for landmark in recent:
                if landmark in point_col:
                    marginals[landmark] = marginal(point_col[landmark], 2)
            if monotonic() - start > budget_s:
                raise TimeoutError("optimization_budget")
            poses, points = unpack(result.x)
            for i, pose in enumerate(poses):
                self.keys[i].pose = np.r_[pose[:2], wrap(pose[2])]
            for i, key in enumerate(point_ids):
                self.points[key] = points[i]
            self.pose_cov = pose_cov
            self.point_cov.update(marginals)
            self.last_error = ""
            return True
        except (TimeoutError, ValueError, RuntimeError, np.linalg.LinAlgError) as error:
            self.last_error = str(error)
            return False
        finally:
            self.optimization_ms = 1000 * (monotonic() - start)

    def correction(self) -> np.ndarray:
        """Return map->odom from the most recent optimized keyframe."""
        from .geometry import compose

        return (
            compose(self.keys[-1].pose, inverse(self.keys[-1].odom))
            if self.keys
            else np.zeros(3)
        )

    def add_key(self, key: Keyframe) -> int:
        """Append a keyframe with a conservative relative odometry factor."""
        if self.keys:
            previous = self.keys[-1]
            delta = between(previous.odom, key.odom)
            distance = np.linalg.norm(delta[:2])
            # Odom poses are correlated; covariance subtraction is not valid.
            # Sum plus distance-dependent process noise is deliberately conservative.
            cov = (
                previous.cov
                + key.cov
                + np.diag(
                    [0.02**2 + (0.04 * distance) ** 2] * 2
                    + [0.01**2 + (0.03 * distance) ** 2]
                )
            )
            self.edges.append(
                (len(self.keys) - 1, len(self.keys), delta, covariance(cov, 3))
            )
        self.keys.append(key)
        return len(self.keys) - 1
