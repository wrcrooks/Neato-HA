"""Odometry + scan matching -> pose; optionally integrates scans into the map."""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

import numpy as np

from ..config import Config
from ..geometry import Pose, odometry_delta, wrap_angle
from ..robot.base import Odometry
from .grid import OccupancyGrid
from .matcher import LikelihoodField, match

log = logging.getLogger(__name__)

MIN_SCORE = 0.30  # below this the match is ignored and odometry is trusted
MIN_POINTS = 40


@dataclass
class StepInfo:
    pose: Pose
    predicted: Pose
    score: float
    used_match: bool
    points: np.ndarray = field(repr=False)


class Mapper:
    def __init__(self, cfg: Config, grid: OccupancyGrid | None = None):
        self.cfg = cfg
        self.grid = grid or OccupancyGrid(cfg.map_resolution_m, cfg.map_size_m)
        self.pose = Pose()
        self.mapping = True  # False = localise only (map frozen)
        self._last_odom: Odometry | None = None
        self.trajectory: list[Pose] = []
        self.last_points = np.zeros((0, 2))
        self.last_score = 0.0
        self.lost_count = 0
        # Every scan integrated into the map, with its pose: lets the whole map be
        # re-optimised afterwards (see slam/refine.py).
        self.keyframes: list[tuple[Pose, np.ndarray]] = []

    @property
    def sensor_robot(self) -> tuple[float, float]:
        return (self.cfg.lidar_offset_x_mm / 1000.0, 0.0)

    def reset_odometry(self, odom: Odometry) -> None:
        self._last_odom = odom

    def set_pose(self, pose: Pose) -> None:
        self.pose = pose

    def predict(self, odom: Odometry) -> Pose:
        if self._last_odom is None:
            self._last_odom = odom
            return self.pose
        dl = (odom.left_mm - self._last_odom.left_mm) / 1000.0
        dr = (odom.right_mm - self._last_odom.right_mm) / 1000.0
        self._last_odom = odom
        self.pose = self.pose.compose(odometry_delta(dl, dr, self.cfg.wheel_base_mm / 1000.0))
        return self.pose

    def _map_ready(self) -> bool:
        return int(self.grid.occupied().sum()) > 30

    def update(self, odom: Odometry, pts: np.ndarray) -> StepInfo:
        """Fuse one stationary scan. `pts` are robot-frame LIDAR points."""
        before = self.pose
        predicted = self.predict(odom)
        moved = before.distance_to(predicted.x, predicted.y)
        turned = abs(wrap_angle(predicted.theta - before.theta))
        used = False
        score = 0.0
        if len(pts) >= MIN_POINTS and self._map_ready():
            lin = 0.10 + 0.25 * moved + 0.05 * turned
            ang = math.radians(4) + 0.15 * turned + 0.3 * moved
            if self.lost_count:
                lin, ang = lin + 0.15 * self.lost_count, ang + math.radians(5) * self.lost_count
            # Odometry uncertainty grows with distance/rotation; widen the prior with it.
            prior_lin = 0.02 + 0.05 * moved + 0.02 * turned + 0.1 * self.lost_count
            prior_ang = math.radians(1.0) + 0.05 * turned + 0.05 * moved + math.radians(3) * self.lost_count
            res = match(self.grid, pts, predicted, lin_window_m=min(lin, 0.6), ang_window_rad=min(ang, math.radians(35)),
                        max_range_m=self.cfg.lidar_max_range_m, prior_lin_m=prior_lin, prior_ang_rad=prior_ang)
            score = res.score
            if score >= MIN_SCORE:
                self.pose = res.pose
                used = True
                self.lost_count = 0
            else:
                self.lost_count = min(self.lost_count + 1, 3)
                log.info("scan match weak (%.2f); using odometry", score)
        if self.mapping and len(pts) and (used or not self._map_ready()):
            # Only extend the map when the pose is trusted (or the map is still being seeded).
            self.grid.integrate(self.pose, pts, self.sensor_robot)
            self.keyframes.append((self.pose, pts.astype(np.float32)))
        self.trajectory.append(self.pose)
        self.last_points = pts
        self.last_score = score
        return StepInfo(self.pose, predicted, score, used, pts)

    def relocalize(self, pts: np.ndarray, guess: Pose, lin_m: float = 1.0, ang_rad: float = math.radians(45)) -> float:
        """Wide search around `guess` (e.g. the dock) on a frozen map. Returns match score."""
        field = LikelihoodField(self.grid, (guess.x, guess.y), lin_m + self.cfg.lidar_max_range_m + 0.5)
        coarse = match(self.grid, pts, guess, lin_window_m=lin_m, ang_window_rad=ang_rad, lin_step_cells=2,
                       ang_step_rad=math.radians(2), field=field, prior_weight=0.01)
        fine = match(self.grid, pts, coarse.pose, lin_window_m=0.1, ang_window_rad=math.radians(3), field=field,
                     prior_weight=0.01)
        best = fine if fine.score >= coarse.score else coarse
        if best.score >= MIN_SCORE:
            self.pose = best.pose
        return best.score
