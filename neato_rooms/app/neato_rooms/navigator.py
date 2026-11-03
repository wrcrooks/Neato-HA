"""Stop-and-scan motion: move a short step, take a scan, update the pose, repeat."""

from __future__ import annotations

import asyncio
import logging
import math
from dataclasses import dataclass, field
from typing import Awaitable, Callable

from .config import Config
from .geometry import Pose, wrap_angle
from .nav.astar import plan_path
from .nav.costmap import CostMap
from .robot.base import MoveResult, Robot, RobotError
from .slam.mapper import Mapper

log = logging.getLogger(__name__)

HEADING_TOL = math.radians(10)
ARRIVE_TOL_M = 0.08
BACKOFF_M = 0.08


class Cancelled(Exception):
    pass


class Blocked(RobotError):
    """Robot is lifted / stuck and needs a human."""


@dataclass
class NavState:
    path: list[tuple[float, float]] = field(default_factory=list)
    obstacles: list[tuple[float, float]] = field(default_factory=list)  # bump locations this mission
    visited: list[tuple[float, float]] = field(default_factory=list)


class Navigator:
    def __init__(
        self,
        cfg: Config,
        robot: Robot,
        mapper: Mapper,
        nogo: Callable[[], list[list[list[float]]]] = lambda: [],
        on_update: Callable[[], Awaitable[None]] | None = None,
    ):
        self.cfg = cfg
        self.robot = robot
        self.mapper = mapper
        self.nogo = nogo
        self.on_update = on_update
        self.state = NavState()
        self.cancel_event = asyncio.Event()

    @property
    def pose(self) -> Pose:
        return self.mapper.pose

    def check_cancel(self) -> None:
        if self.cancel_event.is_set():
            raise Cancelled()

    def costmap(self) -> CostMap:
        return CostMap(
            self.mapper.grid,
            self.cfg.robot_radius_m,
            self.cfg.safety_margin_m,
            nogo=self.nogo(),
            obstacles=self.state.obstacles,
            visited=self.state.visited,
        )

    # -- Sensing ---------------------------------------------------------------

    async def sense(self) -> None:
        scan = await self.robot.scan()
        odom = await self.robot.odometry()
        self.mapper.update(odom, scan.to_points(self.cfg))
        p = self.mapper.pose
        self.state.visited.append((p.x, p.y))
        if self.on_update:
            await self.on_update()

    async def begin(self) -> None:
        """Call once after entering manual mode, before any motion."""
        self.mapper.reset_odometry(await self.robot.odometry())

    # -- Primitives ------------------------------------------------------------

    async def _after_move(self, res: MoveResult) -> MoveResult:
        if res.safety.lifted:
            raise Blocked("wheel lifted — put the robot back on the floor")
        await self.sense()
        if res.safety.bumper or res.safety.stalled:
            p = self.pose
            r = self.cfg.robot_radius_m + 0.03
            self.state.obstacles.append((p.x + r * math.cos(p.theta), p.y + r * math.sin(p.theta)))
            back = await self.robot.forward(-BACKOFF_M, self.cfg.speed_mm_s // 2)
            if back.safety.lifted:
                raise Blocked("wheel lifted")
            await self.sense()
        return res

    async def turn_to(self, theta: float) -> None:
        for _ in range(3):
            self.check_cancel()
            err = wrap_angle(theta - self.pose.theta)
            if abs(err) < math.radians(3):
                return
            await self._after_move(await self.robot.rotate(err, max(80, self.cfg.speed_mm_s * 2 // 3)))

    async def step_toward(self, x: float, y: float) -> MoveResult | None:
        """One stop-and-scan step towards (x, y). Returns None when arrived."""
        self.check_cancel()
        p = self.pose
        dist = p.distance_to(x, y)
        if dist < ARRIVE_TOL_M:
            return None
        heading = math.atan2(y - p.y, x - p.x)
        if abs(wrap_angle(heading - p.theta)) > HEADING_TOL:
            await self.turn_to(heading)
        step = min(dist, self.cfg.max_step_m)
        return await self._after_move(await self.robot.forward(step, self.cfg.speed_mm_s))

    async def drive_line(self, x: float, y: float, max_steps: int = 200) -> bool:
        """Drive straight to (x, y). False if a bump stopped us."""
        for _ in range(max_steps):
            res = await self.step_toward(x, y)
            if res is None:
                return True
            if res.safety.bumper or res.safety.stalled:
                return False
        return False

    async def go_to(self, x: float, y: float, retries: int = 4, tol: float = 0.15) -> bool:
        """Plan with A* and follow; replans after bumps. False if unreachable."""
        for attempt in range(retries):
            self.check_cancel()
            if self.pose.distance_to(x, y) < tol:
                return True
            path = await asyncio.to_thread(plan_path, self.costmap(), (self.pose.x, self.pose.y), (x, y))
            if not path:
                log.info("no path to (%.2f, %.2f) on attempt %d", x, y, attempt)
                return False
            self.state.path = path
            ok = True
            for wx, wy in path:
                if not await self.drive_line(wx, wy):
                    ok = False
                    break
            self.state.path = []
            if ok:
                return True
        return self.pose.distance_to(x, y) < tol

    async def idle_keepalive(self, seconds: float) -> None:
        end = asyncio.get_running_loop().time() + seconds
        while asyncio.get_running_loop().time() < end:
            self.check_cancel()
            await asyncio.sleep(min(2.0, end - asyncio.get_running_loop().time()))
            await self.robot.keepalive()
