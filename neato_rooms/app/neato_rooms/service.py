"""Owns the robot, map and missions; publishes live state to WebSocket listeners."""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import asdict, dataclass, field

from .calibrate import calibrate
from .config import Config
from .geometry import Pose
from .nav.coverage import plan_lanes
from .nav.frontier import choose_goal
from .navigator import Blocked, Cancelled, Navigator
from .robot import Robot, RobotError, RobotStatus, make_robot
from .slam.grid import OccupancyGrid
from .slam.mapper import Mapper
from .store import Store

log = logging.getLogger(__name__)

UNDOCK_M = 0.4
DOCK_APPROACH = (0.45, 0.0)  # in front of the dock, map frame
LOW_BATTERY = 15
MAP_PUSH_INTERVAL_S = 1.5


@dataclass
class MissionInfo:
    kind: str
    status: str = "running"  # running | done | failed | cancelled
    message: str = ""
    progress: float = 0.0
    started: float = field(default_factory=time.time)
    areas: list[str] = field(default_factory=list)


class Service:
    def __init__(self, cfg: Config, robot: Robot | None = None, store: Store | None = None):
        self.cfg = cfg
        self.robot = robot or make_robot(cfg)
        self.store = store or Store(cfg.data_dir)
        grid = self.store.load_grid()
        self.mapper = Mapper(cfg, grid)
        self.mapper.mapping = False
        self.nav = Navigator(cfg, self.robot, self.mapper, nogo=self.store.nogo_polygons, on_update=self._on_nav_update)
        self.mission: MissionInfo | None = None
        self._task: asyncio.Task | None = None
        self._poll_task: asyncio.Task | None = None
        self.status = RobotStatus()
        self._listeners: set[asyncio.Queue] = set()
        self._last_map_push = 0.0
        self._last_map_version = -1
        self._drive_q: asyncio.Queue[tuple[str, float]] = asyncio.Queue()
        self.map_dirty = False  # unsaved mapping changes
        self.calibration: dict | None = None

    # -- Lifecycle -----------------------------------------------------------------

    async def start(self) -> None:
        self._poll_task = asyncio.create_task(self._poll_loop())

    async def stop(self) -> None:
        if self._task and not self._task.done():
            await self.cancel()
        if self._poll_task:
            self._poll_task.cancel()
        await self.robot.close()

    async def _poll_loop(self) -> None:
        while True:
            try:
                if not self.busy:
                    self.status = await self.robot.status()
                    await self.broadcast({"type": "status", **self.snapshot()})
            except Exception:  # never let the poller die
                log.exception("status poll failed")
            await asyncio.sleep(5.0)

    # -- Pub/sub -----------------------------------------------------------------------

    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=50)
        self._listeners.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        self._listeners.discard(q)

    async def broadcast(self, msg: dict) -> None:
        for q in list(self._listeners):
            if q.full():
                try:
                    q.get_nowait()  # drop oldest for slow clients
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(msg)

    def snapshot(self) -> dict:
        return {
            "robot": self.status.as_dict(),
            "mission": asdict(self.mission) if self.mission else None,
            "pose": self.mapper.pose.as_dict(),
            "has_map": self.store.has_map(),
            "map_dirty": self.map_dirty,
            "backend": self.cfg.robot,
            "calibration": self.calibration,
        }

    def live(self) -> dict:
        pts = self.mapper.pose.transform_points(self.mapper.last_points[::2]) if len(self.mapper.last_points) else []
        return {
            "type": "live",
            "pose": self.mapper.pose.as_dict(),
            "score": round(self.mapper.last_score, 2),
            "path": [[round(x, 3), round(y, 3)] for x, y in self.nav.state.path],
            "scan": [[round(float(x), 3), round(float(y), 3)] for x, y in pts],
        }

    def map_message(self) -> dict:
        return {"type": "map", **self.mapper.grid.export()}

    async def _on_nav_update(self) -> None:
        await self.broadcast(self.live())
        now = time.monotonic()
        if self.mapper.grid.version != self._last_map_version and now - self._last_map_push > MAP_PUSH_INTERVAL_S:
            self._last_map_push = now
            self._last_map_version = self.mapper.grid.version
            await self.broadcast(self.map_message())

    # -- Missions --------------------------------------------------------------------

    @property
    def busy(self) -> bool:
        return self._task is not None and not self._task.done()

    def start_mission(self, kind: str, **kw) -> MissionInfo:
        if self.busy:
            raise RuntimeError(f"mission '{self.mission.kind}' already running")
        runner = {
            "explore": self._explore,
            "manual_map": self._manual_map,
            "clean": self._clean,
            "dock": self._go_dock,
            "calibrate": self._calibrate,
        }[kind]
        self.mission = MissionInfo(kind=kind, areas=list(kw.get("area_ids", [])))
        self.nav.cancel_event.clear()
        self._task = asyncio.create_task(self._run(runner, **kw))
        return self.mission

    async def cancel(self) -> None:
        self.nav.cancel_event.set()
        if self._task:
            try:
                await asyncio.wait_for(asyncio.shield(self._task), timeout=15)
            except asyncio.TimeoutError:
                self._task.cancel()

    async def _run(self, runner, **kw) -> None:
        m = self.mission
        await self._push_status()
        try:
            await runner(**kw)
            m.status, m.progress = "done", 1.0
            m.message = m.message or "finished"
        except Cancelled:
            m.status, m.message = "cancelled", "cancelled"
        except Blocked as e:
            m.status, m.message = "failed", str(e)
        except RobotError as e:
            m.status, m.message = "failed", f"robot: {e}"
        except Exception as e:  # pragma: no cover - surfaced to UI
            log.exception("mission crashed")
            m.status, m.message = "failed", f"internal error: {e}"
        finally:
            await self._safe_stop()
            self.nav.state.path = []
            self.mapper.mapping = False
            await self._push_status()

    async def _safe_stop(self) -> None:
        try:
            await self.robot.set_cleaning(False, False, False)
        except Exception:
            pass
        # Always leave TestMode so the robot's own buttons and safety work again.
        try:
            await self.robot.set_manual(False)
        except Exception:
            pass

    async def _push_status(self) -> None:
        try:
            self.status = await self.robot.status()
        except Exception:
            pass
        await self.broadcast({"type": "status", **self.snapshot()})
        await self.broadcast(self.live())

    async def _set_message(self, msg: str, progress: float | None = None) -> None:
        self.mission.message = msg
        if progress is not None:
            self.mission.progress = max(0.0, min(1.0, progress))
        await self.broadcast({"type": "status", **self.snapshot()})

    async def _enter_manual(self) -> None:
        self.status = await self.robot.status()
        await self.robot.set_manual(True)
        await self.nav.begin()

    async def _start_from_dock_or_relocalize(self) -> None:
        """Establish the pose on a saved map before cleaning."""
        scan = await self.robot.scan()
        pts = scan.to_points(self.cfg)
        guess = Pose() if self.status.on_dock else self.mapper.pose
        score = self.mapper.relocalize(pts, guess, lin_m=0.5 if self.status.on_dock else 1.5)
        if score < 0.3:
            raise Blocked(
                "couldn't find the robot on the map — put it on the dock, or drag the robot marker to where it is"
            )
        self.mapper.reset_odometry(await self.robot.odometry())

    async def _undock(self) -> None:
        if self.status.on_dock:
            await self._set_message("undocking")
            await self.nav._after_move(await self.robot.forward(UNDOCK_M, self.cfg.speed_mm_s // 2))

    async def _explore(self, new_map: bool = True, max_goals: int = 200) -> None:
        if new_map:
            self.mapper.grid = OccupancyGrid(self.cfg.map_resolution_m, self.cfg.map_size_m)
            self.mapper.set_pose(Pose())
            self.nav.state.visited.clear()
        self.nav.state.obstacles.clear()
        await self._enter_manual()
        self.mapper.mapping = True
        self.map_dirty = True
        if not new_map:
            await self._start_from_dock_or_relocalize()
        await self.nav.sense()
        await self._undock()
        for _ in range(4):  # look around once
            await self.nav._after_move(await self.robot.rotate(math.pi / 2, 120))
        tried: list[tuple[float, float]] = []
        for i in range(max_goals):
            self.nav.check_cancel()
            await self._check_battery()
            cm = self.nav.costmap()
            goal = await asyncio.to_thread(choose_goal, cm, (self.mapper.pose.x, self.mapper.pose.y), tried)
            if goal is None:
                break
            tried.append(goal)
            await self._set_message(f"exploring (goal {i + 1})")
            await self.nav.go_to(*goal, tol=0.2)
        await self._set_message("exploration finished, returning to dock")
        self.store.save_grid(self.mapper.grid)
        self.map_dirty = False
        await self.broadcast(self.map_message())
        await self._return_and_dock()

    async def _manual_map(self, new_map: bool = True) -> None:
        if new_map:
            self.mapper.grid = OccupancyGrid(self.cfg.map_resolution_m, self.cfg.map_size_m)
            self.mapper.set_pose(Pose())
            self.nav.state.visited.clear()
        await self._enter_manual()
        self.mapper.mapping = True
        self.map_dirty = True
        if not new_map:
            await self._start_from_dock_or_relocalize()
        await self.nav.sense()
        await self.broadcast(self.map_message())
        await self._set_message("drive the robot around with the arrows; press Finish when done")
        while not self._drive_q.empty():
            self._drive_q.get_nowait()
        while True:
            self.nav.check_cancel()
            try:
                cmd, amount = await asyncio.wait_for(self._drive_q.get(), timeout=2.0)
            except asyncio.TimeoutError:
                await self.robot.keepalive()
                continue
            if cmd == "finish":
                break
            if cmd == "forward":
                res = await self.robot.forward(amount, self.cfg.speed_mm_s)
            elif cmd == "rotate":
                res = await self.robot.rotate(amount, 120)
            else:
                continue
            await self.nav._after_move(res)
        self.store.save_grid(self.mapper.grid)
        self.map_dirty = False
        await self.broadcast(self.map_message())
        await self._set_message("map saved")
        await self.robot.set_manual(False)

    def drive(self, cmd: str, amount: float = 0.0) -> None:
        if not (self.busy and self.mission and self.mission.kind == "manual_map"):
            raise RuntimeError("start manual mapping first")
        if cmd == "forward":
            amount = max(-0.5, min(0.5, amount or 0.25))
        elif cmd == "rotate":
            amount = max(-math.pi, min(math.pi, amount or math.pi / 4))
        elif cmd != "finish":
            raise ValueError(cmd)
        self._drive_q.put_nowait((cmd, amount))

    async def _check_battery(self) -> None:
        st = self.status = await self.robot.status()
        if st.battery is not None and st.battery < LOW_BATTERY:
            raise Blocked(f"battery low ({st.battery}%) — dock and recharge")

    async def _clean(self, area_ids: list[str], passes: int = 1) -> None:
        if not self.store.has_map():
            raise Blocked("no map yet — map the house first")
        areas = [self.store.get_area(a) for a in area_ids]
        areas = [a for a in areas if a["kind"] in ("room", "zone")]
        if not areas:
            raise Blocked("pick at least one room or zone")
        self.mapper.grid = self.store.load_grid()
        self.mapper.mapping = False
        self.nav.state.obstacles.clear()
        await self._enter_manual()
        await self._start_from_dock_or_relocalize()
        await self._undock()

        total = len(areas) * passes
        done = 0
        for p in range(passes):
            for area in areas:
                self.nav.check_cancel()
                await self._check_battery()
                await self._set_message(f"going to {area['name']}", done / total)
                lanes = await asyncio.to_thread(
                    plan_lanes, self.nav.costmap(), area["polygon"], self.cfg.lane_spacing_m,
                    (self.mapper.pose.x, self.mapper.pose.y),
                )
                if not lanes:
                    log.warning("area %s has no reachable floor", area["name"])
                    done += 1
                    continue
                if not await self.nav.go_to(*lanes[0].start, tol=0.2):
                    log.warning("could not reach %s", area["name"])
                    done += 1
                    continue
                await self.robot.set_cleaning(True, True, True)
                for i, lane in enumerate(lanes):
                    self.nav.check_cancel()
                    await self._set_message(f"cleaning {area['name']}", (done + i / len(lanes)) / total)
                    if self.mapper.pose.distance_to(*lane.start) > 0.3:
                        if not await self.nav.go_to(*lane.start, tol=0.15):
                            continue
                    await self.nav.drive_line(*lane.start)
                    await self.nav.drive_line(*lane.end)
                await self.robot.set_cleaning(False, False, False)
                done += 1
        await self._return_and_dock()

    async def _calibrate(self) -> None:
        await self._enter_manual()
        if self.status.on_dock:
            await self.robot.forward(UNDOCK_M + 0.1, self.cfg.speed_mm_s // 2)
        self.calibration = await calibrate(self.cfg, self.robot, self._set_message)
        c = self.calibration
        if "error" in c:
            raise Blocked(c["error"])
        parts = [f"wheel_base_mm ≈ {c['wheel_base_mm_suggested']} (configured {c['wheel_base_mm_configured']:g})"]
        if c["lidar_ccw"] != self.cfg.lidar_ccw:
            parts.append(f"set lidar_ccw to {str(c['lidar_ccw']).lower()}")
        self.mission.message = "; ".join(parts)
        await self.robot.set_manual(False)

    async def _go_dock(self) -> None:
        """Drive back on our map if we can localise; otherwise let the firmware find the base."""
        if self.store.has_map():
            self.mapper.grid = self.store.load_grid()
            self.mapper.mapping = False
            await self._enter_manual()
            try:
                await self._start_from_dock_or_relocalize()
            except Blocked:
                await self._set_message("not localised; using the robot's own docking")
                await self.robot.native("dock")
                return
            await self._return_and_dock()
        else:
            await self.robot.native("dock")

    async def _return_and_dock(self) -> None:
        await self._set_message("returning to dock")
        await self.robot.set_cleaning(False, False, False)
        if self.mapper.pose.distance_to(*DOCK_APPROACH) > 0.2:
            await self.nav.go_to(*DOCK_APPROACH, tol=0.2)
        try:
            await self.robot.native("dock")
        except RobotError as e:
            raise Blocked(f"couldn't dock: {e}") from e
        self.mapper.set_pose(Pose())
        await self._set_message("docked")

    # -- Map management -------------------------------------------------------------

    def save_map(self) -> None:
        self.store.save_grid(self.mapper.grid)
        self.map_dirty = False

    def reset_map(self) -> None:
        if self.busy:
            raise RuntimeError("stop the current mission first")
        self.store.delete_map()
        self.mapper.grid = OccupancyGrid(self.cfg.map_resolution_m, self.cfg.map_size_m)
        self.mapper.set_pose(Pose())
        self.map_dirty = False

    def set_pose(self, pose: Pose) -> None:
        self.mapper.set_pose(pose)

    async def native(self, action: str) -> None:
        if self.busy:
            raise RuntimeError("stop the current mission first")
        await self.robot.native(action)
        await self._push_status()
