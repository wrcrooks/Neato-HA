"""Global map refinement: re-align every scan against the map built from all scans.

Online SLAM only ever matches a scan against the map built *so far*, so small errors early
on get baked in: walls seen from two rooms end up a few centimetres apart and blur. Here
all keyframes are re-registered against the complete map (on a finer grid), the map is
rebuilt from the corrected poses, and this repeats until the poses settle. The first
keyframe (the dock) stays fixed, so the map frame doesn't move.
"""

from __future__ import annotations

import math
from typing import Callable

import numpy as np

from ..config import Config
from ..geometry import Pose
from .grid import OccupancyGrid
from .matcher import LikelihoodField, match

Keyframes = list[tuple[Pose, np.ndarray]]


def build_grid(cfg: Config, keyframes: Keyframes, resolution: float | None = None) -> OccupancyGrid:
    g = OccupancyGrid(resolution or cfg.map_resolution_m, cfg.map_size_m)
    sensor = (cfg.lidar_offset_x_mm / 1000.0, 0.0)
    for pose, pts in keyframes:
        g.integrate(pose, pts, sensor)
    return g


def refine(
    cfg: Config,
    keyframes: Keyframes,
    iterations: int = 6,
    resolution: float = 0.03,
    progress: Callable[[float], None] | None = None,
) -> tuple[Keyframes, dict]:
    if len(keyframes) < 3:
        return keyframes, {"iterations": 0}
    kfs = list(keyframes)
    history = []
    for it in range(iterations):
        grid = build_grid(cfg, kfs, resolution)
        x0, y0, x1, y1 = grid.known_bbox(pad=0)
        cx, cy = grid.cell_to_world((x0 + x1) / 2, (y0 + y1) / 2)
        half = max(x1 - x0, y1 - y0) * resolution / 2 + 1.0
        field = LikelihoodField(grid, (float(cx), float(cy)), half, sigma_m=0.04)
        moved = []
        out = [kfs[0]]
        for i, (pose, pts) in enumerate(kfs[1:], start=1):
            r = match(grid, pts, pose, lin_window_m=0.08, ang_window_rad=math.radians(1.5),
                      ang_step_rad=math.radians(0.25), field=field, prior_weight=0.005)
            if r.score >= 0.3:
                moved.append(math.hypot(r.pose.x - pose.x, r.pose.y - pose.y))
                out.append((r.pose, pts))
            else:
                out.append((pose, pts))
            if progress and i % 50 == 0:
                progress((it + i / len(kfs)) / iterations)
        kfs = out
        mean_move = float(np.mean(moved)) if moved else 0.0
        history.append(round(mean_move, 4))
        if mean_move < 0.002:
            break
    return kfs, {"iterations": len(history), "mean_correction_m": history}
