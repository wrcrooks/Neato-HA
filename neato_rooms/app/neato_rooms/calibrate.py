"""On-robot calibration using the LIDAR as ground truth.

1. LIDAR direction: rotate +90 deg and check which angle convention makes the new scan
   line up with the first one.
2. Wheel base: command full 360 deg spins; the scan-matched residual rotation tells us
   how far off the configured wheel base is.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Awaitable, Callable

from .config import Config
from .geometry import Pose
from .robot.base import Robot, Scan
from .slam.grid import OccupancyGrid
from .slam.matcher import match


def _grid_from(scan: Scan, cfg: Config) -> OccupancyGrid:
    g = OccupancyGrid(cfg.map_resolution_m, 2 * cfg.lidar_max_range_m + 2)
    pts = scan.to_points(cfg)
    for _ in range(3):  # make the walls solid
        g.integrate(Pose(), pts, (cfg.lidar_offset_x_mm / 1000.0, 0.0))
    return g


def _heading(grid: OccupancyGrid, scan: Scan, cfg: Config, guess: float, window_deg: float) -> tuple[float, float]:
    r = match(grid, scan.to_points(cfg), Pose(0, 0, guess), lin_window_m=0.08,
              ang_window_rad=math.radians(window_deg), ang_step_rad=math.radians(0.5), prior_weight=0.0)
    return r.pose.theta, r.score


async def calibrate(
    cfg: Config, robot: Robot, report: Callable[[str, float], Awaitable[None]], spins: int = 2
) -> dict:
    speed = 100
    result: dict = {"wheel_base_mm_configured": cfg.wheel_base_mm}

    # -- LIDAR direction -------------------------------------------------------
    await report("checking LIDAR direction", 0.05)
    s0 = await robot.scan()
    await robot.rotate(math.pi / 2, speed)
    s1 = await robot.scan()
    scores = {}
    for ccw in (True, False):
        c = dataclasses.replace(cfg, lidar_ccw=ccw)
        _, scores[ccw] = _heading(_grid_from(s0, c), s1, c, math.pi / 2, 20)
    lidar_ccw = scores[True] >= scores[False]
    result["lidar_ccw"] = lidar_ccw
    result["lidar_direction_scores"] = {"ccw": round(scores[True], 3), "cw": round(scores[False], 3)}
    await robot.rotate(-math.pi / 2, speed)

    # -- Wheel base ----------------------------------------------------------------
    c = dataclasses.replace(cfg, lidar_ccw=lidar_ccw)
    ratios = []
    for i in range(spins):
        await report(f"spin {i + 1}/{spins}", 0.2 + 0.7 * i / spins)
        before = await robot.scan()
        grid = _grid_from(before, c)
        # Command one full turn in four quarter turns (fewer coasting errors on real robots).
        for _ in range(4):
            await robot.rotate(math.pi / 2, speed)
        after = await robot.scan()
        residual, score = _heading(grid, after, c, 0.0, 25)
        if score < 0.3:
            continue
        actual = 2 * math.pi + residual  # over-rotation shows up as positive residual
        ratios.append((2 * math.pi) / actual)
    if not ratios:
        result["error"] = "scans did not line up; try somewhere with more walls nearby"
        return result
    ratio = sum(ratios) / len(ratios)
    # Commanded arc = theta_cmd * wb_cfg / 2 and actual theta = arc * 2 / wb_true,
    # so theta_cmd / theta_actual = wb_true / wb_cfg.
    result["wheel_base_mm_suggested"] = round(cfg.wheel_base_mm * ratio, 1)
    result["spins_used"] = len(ratios)
    return result
