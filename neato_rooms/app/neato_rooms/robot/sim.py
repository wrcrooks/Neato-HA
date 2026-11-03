"""Simulated D7: a 2-D house of wall segments, ray-cast LIDAR, noisy odometry, bumpers.

Lets the whole stack (mapping, rooms, cleaning, PWA) be exercised without hardware.
"""

from __future__ import annotations

import asyncio
import math

import numpy as np

from ..config import Config
from ..geometry import Pose, odometry_delta, wrap_angle
from .base import MoveResult, Odometry, Robot, RobotError, RobotStatus, Safety, Scan


def _rect(x0, y0, x1, y1):
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def _loop(pts):
    return [(*pts[i], *pts[(i + 1) % len(pts)]) for i in range(len(pts))]


INTERIOR_WALL_M = 0.12
EXTERIOR_WALL_M = 0.20


def default_house() -> tuple[np.ndarray, Pose]:
    """~9 x 6 m flat: living room, kitchen, bedroom, hallway; returns (segments, dock pose).

    Walls have real thickness (interior 12 cm, centred on the lines below; exterior 20 cm,
    outside the 0..9 x 0..6 interior), and the living room has a 45-degree corner.

    Layout (world frame, metres; interior faces):
        y=6 +-----------------+---------+
            |  bedroom        | kitchen |
        y=3.4+---- ---+ hall  +--- -----+
            |  living room             \\
        y=0 +-----------------------+---+  (corner cut 8.4,0 -> 9,0.6)
            x=0                      x=9
    """
    s: list[tuple] = []
    inner = [(0, 0), (8.4, 0), (9, 0.6), (9, 6), (0, 6)]
    e = EXTERIOR_WALL_M
    outer = [(-e, -e), (8.4 + e * 0.414, -e), (9 + e, 0.6 - e * 0.414), (9 + e, 6 + e), (-e, 6 + e)]
    s += _loop(inner) + _loop(outer)
    h = INTERIOR_WALL_M / 2
    # Wall at y=3.4 with doorways x 2.4-3.2 (bedroom) and 6.6-7.4 (kitchen); the hallway
    # opening x 4.4-5.4 has no wall. Pieces next to the hallway walls overlap them.
    for x0, x1 in [(0, 2.4), (3.2, 4.4 + h), (5.4 - h, 6.6), (7.4, 9)]:
        s += _rect(x0, 3.4 - h, x1, 3.4 + h)
    # Hallway walls separating the bedroom (x<4.4) and kitchen (x>5.4).
    s += _rect(4.4 - h, 3.4 - h, 4.4 + h, 6.0) + _rect(5.4 - h, 3.4 - h, 5.4 + h, 6.0)
    # Furniture: sofa, kitchen island, bed (as boxes), table leg-ish posts.
    s += _rect(0.0, 2.2, 2.0, 3.0)  # sofa against the dividing wall
    s += _rect(6.8, 4.4, 7.8, 5.0)  # kitchen island
    s += _rect(0.0, 4.4, 2.0, 6.0)  # bed against the outer wall
    s += _rect(5.0, 1.2, 5.12, 1.32) + _rect(6.0, 1.2, 6.12, 1.32)  # table legs
    s += _rect(5.0, 2.0, 5.12, 2.12) + _rect(6.0, 2.0, 6.12, 2.12)
    dock = Pose(0.5, 0.6, 0.0)  # dock against the left wall, robot facing +x
    return np.array(s, dtype=float), dock


def _seg_dist(px: float, py: float, segs: np.ndarray) -> float:
    ax, ay, bx, by = segs.T
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / np.where(l2 == 0, 1, l2), 0, 1)
    cx, cy = ax + t * dx, ay + t * dy
    return float(np.min(np.hypot(px - cx, py - cy)))


class SimRobot(Robot):
    def __init__(
        self,
        cfg: Config,
        segments: np.ndarray | None = None,
        start: Pose | None = None,
        seed: int | None = 1,
        odom_noise: float = 0.01,
        true_wheel_base_mm: float | None = None,
        range_noise_mm: float = 8.0,
    ):
        super().__init__(cfg)
        house, dock = default_house()
        self.segments = house if segments is None else segments
        self.dock = dock if start is None else start
        self.pose = self.dock  # ground truth, world frame
        self.rng = np.random.default_rng(seed)
        self.odom_noise = odom_noise
        self.true_wheel_base_mm = true_wheel_base_mm or cfg.wheel_base_mm * 1.01
        self.range_noise_mm = range_noise_mm
        self._enc = [0.0, 0.0]
        self._manual = False
        self._cleaning = (False, False, False)
        self.battery = 100.0
        self.bumped = False
        self.swept: list[tuple[float, float]] = []  # ground-truth positions while vacuum is on
        self.state = "ST_C_Standby"

    # -- Robot API --------------------------------------------------------------

    async def status(self) -> RobotStatus:
        on_dock = self.pose.distance_to(self.dock.x, self.dock.y) < 0.05
        return RobotStatus(
            connected=True,
            battery=int(self.battery),
            charging=on_dock and self.battery < 100,
            on_dock=on_dock,
            state=self.state,
            manual=self._manual,
        )

    async def set_manual(self, enable: bool) -> None:
        self._manual = enable
        self.state = "ST_TestMode" if enable else "ST_C_Standby"
        if not enable:
            self._cleaning = (False, False, False)

    async def odometry(self) -> Odometry:
        return Odometry(*self._enc)

    async def scan(self) -> Scan:
        if not self._manual:
            raise RobotError("LIDAR only spins in manual mode")
        cfg = self.cfg
        p = self.pose
        lo = cfg.lidar_offset_x_mm / 1000.0
        ox, oy = p.x + lo * math.cos(p.theta), p.y + lo * math.sin(p.theta)
        ang = np.arange(360)
        sign = 1.0 if cfg.lidar_ccw else -1.0
        rel = np.radians(sign * ang + cfg.lidar_angle_offset_deg)
        dirs = np.stack([np.cos(p.theta + rel), np.sin(p.theta + rel)], axis=1)
        ax, ay, bx, by = self.segments.T
        ex, ey = bx - ax, by - ay
        # Solve o + t*d = a + u*e for each ray/segment pair.
        dxr, dyr = dirs[:, 0:1], dirs[:, 1:2]
        den = dxr * ey - dyr * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((ax - ox) * ey - (ay - oy) * ex) / den
            u = ((ax - ox) * dyr - (ay - oy) * dxr) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        t = np.where(hit, t, np.inf).min(axis=1)
        dist = t * 1000.0 + self.rng.normal(0, self.range_noise_mm, 360)
        err = np.zeros(360, dtype=int)
        bad = ~np.isfinite(t) | (t > cfg.lidar_max_range_m) | (self.rng.random(360) < 0.02)
        err[bad] = 0x8035
        dist = np.where(bad, 0, np.round(dist)).astype(int)
        await self._sleep(0.2)
        return Scan(ang, dist, err)

    async def move(self, left_mm: float, right_mm: float, speed_mm_s: int) -> MoveResult:
        if not self._manual:
            raise RobotError("move() requires manual mode")
        steps = max(1, int(max(abs(left_mm), abs(right_mm)) / 10.0))
        dl, dr = left_mm / steps, right_mm / steps
        wb = self.true_wheel_base_mm / 1000.0
        done_l = done_r = 0.0
        bumped = False
        for _ in range(steps):
            # Wheel slip: true motion deviates from encoder ticks.
            tl = dl * (1 + self.rng.normal(0, self.odom_noise)) / 1000.0
            tr = dr * (1 + self.rng.normal(0, self.odom_noise)) / 1000.0
            nxt = self.pose.compose(odometry_delta(tl, tr, wb))
            if _seg_dist(nxt.x, nxt.y, self.segments) < self.cfg.robot_radius_m:
                bumped = True
                break
            self.pose = nxt
            done_l += dl
            done_r += dr
            if self._cleaning[1]:
                self.swept.append((nxt.x, nxt.y))
        self._enc[0] += done_l
        self._enc[1] += done_r
        self.battery = max(0.0, self.battery - 0.0005 * (abs(done_l) + abs(done_r)) / 10)
        await self._sleep(max(abs(done_l), abs(done_r)) / max(speed_mm_s, 1) + 0.1)
        return MoveResult(done_l, done_r, Safety(bumper=bumped))

    async def set_cleaning(self, brush: bool, vacuum: bool, side_brush: bool) -> None:
        self._cleaning = (brush, vacuum, side_brush)

    async def native(self, action: str) -> None:
        if action == "dock":
            if self.pose.distance_to(self.dock.x, self.dock.y) > 1.5:
                raise RobotError("sim: too far from base to dock")
            await self.set_manual(False)
            self.pose = self.dock
            self.state = "ST_C_Standby"
        elif action in ("house", "spot"):
            self.state = "ST_C_HouseCleaning" if action == "house" else "ST_C_SpotCleaning"
        elif action in ("stop", "pause"):
            self.state = "ST_C_Standby"
        else:
            raise ValueError(action)

    # -- Helpers ---------------------------------------------------------------

    async def _sleep(self, sim_seconds: float) -> None:
        if self.cfg.sim_time_scale > 0:
            await asyncio.sleep(sim_seconds * self.cfg.sim_time_scale)
        else:
            await asyncio.sleep(0)

    def world_to_map(self, p: Pose) -> Pose:
        """Ground truth in the map frame (map origin = dock pose). Test helper."""
        d = self.dock
        c, s = math.cos(-d.theta), math.sin(-d.theta)
        x, y = p.x - d.x, p.y - d.y
        return Pose(c * x - s * y, s * x + c * y, wrap_angle(p.theta - d.theta))
