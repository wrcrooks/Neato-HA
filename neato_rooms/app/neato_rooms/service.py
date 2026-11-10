"""Owns the robot, map and missions; publishes live state to WebSocket listeners."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import time
from dataclasses import asdict, dataclass, field

from .calibrate import calibrate
from .config import Config
from .floorplan import FloorPlan, extract, face_override, layout, same_override, to_pdf, to_svg
from .geometry import Pose, wrap_angle
from .nav.coverage import plan_lanes, viewpoints
from .nav.frontier import choose_goal
from .navigator import Blocked, Cancelled, Navigator
from .robot import Robot, RobotError, RobotStatus, make_robot
from .slam.grid import OccupancyGrid
from .slam.mapper import Mapper
from .slam.refine import build_grid, refine
from .slam.sessions import (align_session, change_record, compose, describe, detect_changes, from_record,
                            transform_keyframes)
from .store import Store

log = logging.getLogger(__name__)

UNDOCK_M = 0.4
DOCK_APPROACH = (0.45, 0.0)  # in front of the dock, map frame
LOW_BATTERY = 15
RESCAN_SPACING_M = 1.5


class LowBattery(Blocked):
    pass
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
        self._fp_cache: dict[bool, tuple[str, FloorPlan]] = {}
        self._fp_lock = asyncio.Lock()
        self._session: dict | None = None  # the mapping run in progress
        self._preview: tuple[str, OccupancyGrid] | None = None

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
            "review": self._review_summary(),
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
            "refine": self._refine,
            "rescan": self._rescan,
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
        await self._start_session("explore", new_map)
        self.nav.state.obstacles.clear()
        await self.nav.sense()
        await self._undock()
        for _ in range(4):  # look around once
            await self.nav._after_move(await self.robot.rotate(math.pi / 2, 120))
        tried: list[tuple[float, float]] = []
        note = "exploration finished"
        for i in range(max_goals):
            self.nav.check_cancel()
            try:
                await self._check_battery()
            except LowBattery as e:
                note = f"{e}; the map so far is kept, extend it after charging"
                break
            cm = self.nav.costmap()
            goal = await asyncio.to_thread(choose_goal, cm, (self.mapper.pose.x, self.mapper.pose.y), tried)
            if goal is None:
                break
            tried.append(goal)
            await self._set_message(f"exploring (goal {i + 1})")
            await self.nav.go_to(*goal, tol=0.2)
        await self._set_message(f"{note}, returning to dock")
        await self._finish_session()
        await self._return_and_dock()
        await self._after_session()

    async def _manual_map(self, new_map: bool = True) -> None:
        await self._start_session("manual", new_map)
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
        await self._finish_session()
        await self.robot.set_manual(False)
        await self._after_session()

    async def _rescan(self, area_ids: list[str] | None = None) -> None:
        """Re-visit the mapped floor to sharpen the map and pick up rearranged furniture."""
        if not self.store.has_map():
            raise Blocked("no map yet — map the house first")
        polys = [self.store.get_area(a)["polygon"] for a in area_ids or []]
        await self._start_session("rescan", new_map=False, stitch=False)
        self.nav.state.obstacles.clear()
        await self.nav.sense()
        await self._undock()
        spots = await asyncio.to_thread(
            viewpoints, self.nav.costmap(), (self.mapper.pose.x, self.mapper.pose.y), RESCAN_SPACING_M, polys or None
        )
        note = "re-scan finished"
        for i, spot in enumerate(spots):
            self.nav.check_cancel()
            try:
                await self._check_battery()
            except LowBattery as e:
                note = f"{e}; the part re-scanned so far is kept"
                break
            await self._set_message(f"re-scanning ({i + 1}/{len(spots)})", 0.9 * i / max(len(spots), 1))
            await self.nav.go_to(*spot, tol=0.3)
        await self._set_message(f"{note}, returning to dock")
        await self._finish_session()
        await self._return_and_dock()
        await self._after_session()

    # -- Mapping sessions ------------------------------------------------------------------

    async def _start_session(self, kind: str, new_map: bool, stitch: bool = True) -> None:
        """Set up the mapper for a mapping run and enter manual mode.

        Extending: the robot is placed on the saved map from the dock. Started anywhere else
        (or if that fails), a run that allows `stitch` is mapped on its own and stitched onto
        the map when it ends; a re-scan instead relocalises near the last known pose.
        """
        if new_map:
            self.mapper.grid = OccupancyGrid(self.cfg.map_resolution_m, self.cfg.map_size_m)
            self.mapper.set_pose(Pose())
            self.mapper.keyframes = []
            self.nav.state.visited.clear()
            self._session = {"id": 0, "kind": kind, "started": time.time(), "base": 0, "detached": False}
        else:
            if self.store.pending_session():
                raise Blocked("review the changes from the last scan first (Map → Review changes)")
            if not self.store.has_map():
                raise Blocked("no map to extend — start a new one")
            kfs = self.store.load_scans()
            if not kfs:
                raise Blocked("this map was saved without its scans, so it can't be extended — start a new map")
            sid = max(x["id"] for x in self.store.sessions()) + 1
            self.mapper.grid = self.store.load_grid()
            self.mapper.keyframes = list(kfs)
            self._session = {"id": sid, "kind": kind, "started": time.time(), "base": len(kfs), "detached": False}
        await self._enter_manual()
        self.mapper.mapping = True
        self.map_dirty = True
        if new_map:
            return
        try:
            # Off the dock, a quick match near the last known pose can lock onto the wrong spot;
            # a mapping run that starts there is mapped on its own and stitched on reliably later.
            if stitch and not self.status.on_dock:
                raise Blocked("not on the dock")
            await self._start_from_dock_or_relocalize()
        except Blocked:
            if not stitch:
                raise
            # Map this run on its own; it's stitched onto the map when it ends.
            self._session.update(detached=True, base=0)
            self.mapper.grid = OccupancyGrid(self.cfg.map_resolution_m, self.cfg.map_size_m)
            self.mapper.set_pose(Pose())
            self.mapper.keyframes = []
            self.nav.state.visited.clear()
            self.mapper.reset_odometry(await self.robot.odometry())
            await self._set_message("couldn't place the robot on the map; mapping on its own, to stitch on afterwards")

    async def _finish_session(self) -> None:
        """Save this run's scans (robot still in manual mode, wherever it is).

        A new map is saved as is. An extension or re-scan is saved as *pending*: the map
        stays as it was until its changes are reviewed. A run mapped on its own is first
        stitched onto the map, and the robot's pose moved into the map frame.
        """
        sess = self._session
        new = self.mapper.keyframes[sess["base"]:]
        record = {"id": sess["id"], "kind": sess["kind"], "started": sess["started"], "status": "applied",
                  "scans": len(new), "stitched": sess["detached"], "changes": []}
        if sess["id"] == 0:
            self.store.save_grid(self.mapper.grid)
            self.store.save_scans(new, [0] * len(new))
            self.store.save_sessions([record])
        else:
            kfs, ids = self.store.load_scans(), list(self.store.load_scan_ids())
            if sess["detached"]:
                await self._set_message("stitching the new scans onto the map")
                found = await asyncio.to_thread(align_session, self.cfg, self.store.load_grid(), new)
                if found is None:
                    raise Blocked("couldn't stitch the new scans onto the map: they don't overlap it enough. "
                                  "Start the next run from the dock, or somewhere already mapped")
                t, overlap = found
                log.info("stitched session %d: %s (overlap %.2f)", sess["id"], t, overlap)
                new = transform_keyframes(new, t)
                self.mapper.set_pose(t.compose(self.mapper.pose))
                st = self.nav.state
                st.visited = [(p.x, p.y) for p in (t.compose(Pose(x, y)) for x, y in st.visited)]
                st.obstacles = [(p.x, p.y) for p in (t.compose(Pose(x, y)) for x, y in st.obstacles)]
                self.mapper.keyframes = new
                self.mapper.grid = await asyncio.to_thread(build_grid, self.cfg, kfs + new)
                await self.broadcast(self.map_message())
            self.store.save_scans(kfs + new, ids + [sess["id"]] * len(new))
            self.store.save_sessions(self.store.sessions() + [{**record, "status": "pending"}])
        self.map_dirty = False

    async def _after_session(self) -> None:
        """With the robot idle: refine every scan, then look for changes in the new run."""
        await self._refine_stored()
        pend = self.store.pending_session()
        if not pend:
            self.mission.message = "map saved"
            return
        await self._set_message("looking for changes", 1.0)
        kfs, ids = self.store.load_scans(), self.store.load_scan_ids()
        applied = [x for x in self.store.sessions() if x["status"] == "applied"]

        def work():
            before = compose(self.cfg, kfs, ids, applied)
            after = build_grid(self.cfg, [k for k, i in zip(kfs, ids) if i == pend["id"]])
            return detect_changes(before, after)

        changes = await asyncio.to_thread(work)
        sessions = self.store.sessions()
        for x in sessions:
            if x["id"] == pend["id"]:
                x["changes"] = [change_record(n, c) for n, c in enumerate(changes)]
        self.store.save_sessions(sessions)
        if not changes and not pend["stitched"]:
            await self.apply_review()
            self.mission.message = "no changes found; the new scans sharpened the map"
        elif not changes:
            # A stitched run is always checked: a wrong fit would garble the map.
            self.mission.message = "new scans stitched onto the map — check they line up under Map → Review changes"
        else:
            self.mission.message = (f"{len(changes)} change{'s' if len(changes) > 1 else ''} found — "
                                    "review them under Map → Review changes")
        await self.broadcast({"type": "review"})

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
            raise LowBattery(f"battery low ({st.battery}%) — dock and recharge")

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

    async def _refine(self) -> None:
        if self.store.pending_session():
            raise Blocked("review the changes from the last scan first (Map → Review changes)")
        if len(self.store.load_scans()) < 3:
            raise Blocked("no scans saved for this map — map the house again to refine it")
        self.mapper.grid = self.store.load_grid() or self.mapper.grid
        await self._refine_stored()
        self.mission.message = "map refined"

    async def _refine_stored(self) -> None:
        """Re-align every saved scan against the whole map, then rebuild and save the map.

        Runs with the robot idle (docked / out of TestMode): it takes a while on small hardware.
        """
        kfs, ids = self.store.load_scans(), self.store.load_scan_ids()
        if len(kfs) < 3:
            return
        loop = asyncio.get_running_loop()

        def progress(f: float) -> None:
            self.mission.progress = f
            asyncio.run_coroutine_threadsafe(self._set_message("refining map", f), loop)

        await self._set_message("refining map", 0.0)
        new, info = await asyncio.to_thread(refine, self.cfg, kfs, progress=progress)
        log.info("map refined: %s", info)
        # Keep the robot where it is relative to the corrected scans.
        (old_last, _), (new_last, _) = kfs[-1], new[-1]
        p = self.mapper.pose
        c, s = math.cos(-old_last.theta), math.sin(-old_last.theta)
        dx, dy = p.x - old_last.x, p.y - old_last.y
        rel = Pose(c * dx - s * dy, s * dx + c * dy, wrap_angle(p.theta - old_last.theta))
        self.mapper.keyframes = new
        self.mapper.set_pose(new_last.compose(rel))
        self.store.save_scans(new, ids)
        await self._rebuild_map()

    async def _rebuild_map(self) -> None:
        """The saved map = every applied session's scans, with their change decisions."""
        kfs, ids = self.store.load_scans(), self.store.load_scan_ids()
        applied = [x for x in self.store.sessions() if x["status"] == "applied"]
        if not kfs or not applied:
            return
        grid = await asyncio.to_thread(compose, self.cfg, kfs, ids, applied)
        grid.version = self.mapper.grid.version + 1
        self.mapper.grid = grid
        self.store.save_grid(grid)
        self._preview = None
        await self.broadcast(self.map_message())

    # -- Reviewing a scan's changes -----------------------------------------------------------

    def _review_summary(self) -> dict | None:
        p = self.store.pending_session()
        if not p:
            return None
        return {"session": p["id"], "kind": p["kind"], "changes": len(p["changes"]),
                "accepted": sum(c["accept"] for c in p["changes"])}

    def review(self) -> dict | None:
        """The pending scan's changes, described for the UI (map frame)."""
        p = self.store.pending_session()
        if not p:
            return None
        geo = self.mapper.grid  # every grid has the same cell layout
        changes = [{"id": c["id"], "accept": c["accept"], **describe(from_record(c), geo)} for c in p["changes"]]
        session = {k: v for k, v in p.items() if k != "changes"}
        return {"session": session, "changes": changes}

    def review_decide(self, change_id: int, accept: bool) -> dict:
        sessions = self.store.sessions()
        p = next((x for x in sessions if x["status"] == "pending"), None)
        if not p:
            raise LookupError("nothing to review")
        for c in p["changes"]:
            if c["id"] == change_id:
                c["accept"] = bool(accept)
                self.store.save_sessions(sessions)
                return {"id": change_id, "accept": c["accept"]}
        raise KeyError(f"no change {change_id}")

    async def preview_grid(self) -> OccupancyGrid:
        """The map as it would be with the pending scan applied, as currently decided."""
        p = self.store.pending_session()
        if not p:
            return self.store.load_grid()
        key = json.dumps([self.store.meta().get("saved"), p["id"], [c["accept"] for c in p["changes"]]])
        if self._preview and self._preview[0] == key:
            return self._preview[1]
        kfs, ids = self.store.load_scans(), self.store.load_scan_ids()
        use = [x for x in self.store.sessions() if x["status"] == "applied" or x["id"] == p["id"]]
        grid = await asyncio.to_thread(compose, self.cfg, kfs, ids, use)
        self._preview = (key, grid)
        return grid

    async def apply_review(self) -> None:
        """Merge the pending scan into the map, honouring each change's accept / reject."""
        if self.busy and not (self.mission and self.mission.kind in ("explore", "manual_map", "rescan")):
            raise RuntimeError("stop the current mission first")
        sessions = self.store.sessions()
        p = next((x for x in sessions if x["status"] == "pending"), None)
        if not p:
            raise LookupError("nothing to review")
        p["status"] = "applied"
        self.store.save_sessions(sessions)
        await self._rebuild_map()

    async def discard_review(self) -> None:
        """Drop the pending scan entirely: its scans are deleted and the map is unchanged."""
        if self.busy:
            raise RuntimeError("stop the current mission first")
        sessions = self.store.sessions()
        p = next((x for x in sessions if x["status"] == "pending"), None)
        if not p:
            raise LookupError("nothing to review")
        kfs, ids = self.store.load_scans(), self.store.load_scan_ids()
        keep = [i for i, sid in enumerate(ids) if sid != p["id"]]
        self.store.save_scans([kfs[i] for i in keep], [int(ids[i]) for i in keep])
        self.store.save_sessions([x for x in sessions if x["id"] != p["id"]])
        self.mapper.keyframes = [kfs[i] for i in keep]
        self._preview = None

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
        self.mapper.keyframes = []
        self._fp_cache = {}
        self._preview = None
        self.mapper.set_pose(Pose())
        self.map_dirty = False

    def set_pose(self, pose: Pose) -> None:
        self.mapper.set_pose(pose)

    # -- Floor plan -------------------------------------------------------------------

    async def floorplan(self, preview: bool = False) -> FloorPlan:
        """The listing-style plan of the saved map (cached until the map or settings change).

        preview: the plan as it would be with the pending scan's changes applied.
        """
        if not self.store.has_map():
            raise LookupError("no map yet — map the house first")
        s = self.store.floorplan_settings()
        rooms = [a for a in self.store.areas if a["kind"] == "room"]
        p = self.store.pending_session() if preview else None
        decisions = [p["id"], [c["accept"] for c in p["changes"]]] if p else None
        key = json.dumps([self.store.meta().get("saved"), s["rotation"], s["overrides"], rooms, decisions], sort_keys=True)
        async with self._fp_lock:
            hit = self._fp_cache.get(preview)
            if hit and hit[0] == key:
                return hit[1]
            grid = await self.preview_grid() if p else self.store.load_grid()
            plan = await asyncio.to_thread(extract, grid, rooms, s["rotation"], s["overrides"])
            self._fp_cache[preview] = (key, plan)
            return plan

    async def floorplan_export(self, fmt: str) -> bytes:
        plan = await self.floorplan()
        s = self.store.floorplan_settings()
        page = layout(plan, s["units"], s["title"], s["paper"])
        if fmt == "svg":
            return to_svg(page).encode()
        return await asyncio.to_thread(to_pdf, page, s["title"] or "Floor Plan")

    async def floorplan_align(self, mode: str, face_id: int | None = None, delta_deg: float = 0.0) -> dict:
        s = self.store.floorplan_settings()
        if mode == "auto":
            s["rotation"] = None
        else:
            plan = await self.floorplan()
            if mode == "face":
                f = self._face(plan, face_id)
                s["rotation"] = (f.angle + plan.rotation) % (2 * math.pi)
            elif mode == "rotate":
                s["rotation"] = (plan.rotation + math.radians(delta_deg)) % (2 * math.pi)
            else:
                raise ValueError(f"unknown align mode {mode}")
        return self.store.save_floorplan_settings(s)

    async def floorplan_toggle(self, face_id: int) -> dict:
        plan = await self.floorplan()
        f = self._face(plan, face_id)
        o = face_override(f, plan.rotation, not f.visible)
        s = self.store.floorplan_settings()
        # Replace any earlier override for the same face.
        s["overrides"] = [x for x in s["overrides"] if "normal" in x and not same_override(x, o)] + [o]
        return self.store.save_floorplan_settings(s)

    @staticmethod
    def _face(plan: FloorPlan, face_id: int | None):
        for f in plan.faces:
            if f.id == face_id:
                return f
        raise KeyError(f"no face {face_id}")

    async def native(self, action: str) -> None:
        if self.busy:
            raise RuntimeError("stop the current mission first")
        await self.robot.native(action)
        await self._push_status()
