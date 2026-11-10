"""Several mapping sessions: stitching a separately-mapped run, change detection, re-scan + review."""

import asyncio
import math
import time

import numpy as np
import pytest
from fastapi.testclient import TestClient

from neato_rooms.api import create_app
from neato_rooms.config import Config
from neato_rooms.floorplan import extract
from neato_rooms.geometry import Pose
from neato_rooms.robot.sim import SimRobot, _rect, _seg_dist, default_house
from neato_rooms.service import Service
from neato_rooms.slam.refine import build_grid
from neato_rooms.slam.sessions import align_session, change_record, compose, describe, detect_changes
from neato_rooms.store import Store

SOFA = (0.0, 2.2, 2.0, 3.0)
SOFA_MOVED = (5.5, 0.0, 7.5, 0.8)  # against the front wall
CHAIR = (2.5, 1.0, 3.3, 1.8)
FURNITURE = [SOFA, (6.8, 4.4, 7.8, 5.0), (0.0, 4.4, 2.0, 6.0), SOFA_MOVED, CHAIR]


def house(rearranged: bool):
    segs, _ = default_house()
    segs = [tuple(s) for s in segs]
    if rearranged:
        sofa = _rect(*SOFA)
        segs = [s for s in segs if s not in sofa] + _rect(*SOFA_MOVED) + _rect(*CHAIR)
    return np.array(segs, dtype=float)


def lattice_scans(cfg, rearranged, x_range=(0.3, 9.0), y_range=(0.3, 6.0), frame: Pose | None = None, step=0.5,
                  segs=None):
    """Keyframes from known poses (map frame, or `frame`-relative when given)."""
    segs = house(rearranged) if segs is None else segs
    robot = SimRobot(cfg, segments=segs, seed=3)
    robot._manual = True
    kfs = []
    for x in np.arange(*x_range, step):
        for y in np.arange(*y_range, step):
            if _seg_dist(x, y, segs) < 0.25 or any(a <= x <= c and b <= y <= d for a, b, c, d in FURNITURE):
                continue
            robot.pose = Pose(float(x), float(y), 0.3)
            pts = asyncio.run(robot.scan()).to_points(cfg).astype(np.float32)
            p = robot.world_to_map(robot.pose)
            if frame is not None:
                c, s = math.cos(-frame.theta), math.sin(-frame.theta)
                dx, dy = p.x - frame.x, p.y - frame.y
                p = Pose(c * dx - s * dy, s * dx + c * dy, p.theta - frame.theta)
            kfs.append((p, pts))
    return kfs, robot


def world_box_in_map(robot, box):
    x0, y0, x1, y1 = box
    pts = [robot.world_to_map(Pose(x, y, 0)) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
    return min(p.x for p in pts), min(p.y for p in pts), max(p.x for p in pts), max(p.y for p in pts)


def occupied_at(grid, points):
    """How many of these places (a change's cells, as world points) are occupied."""
    c = grid.world_to_cell(np.array(points))
    return int(grid.occupied()[c[:, 1], c[:, 0]].sum())


def inside(xy, box, tol=0.3):
    return box[0] - tol <= xy[0] <= box[2] + tol and box[1] - tol <= xy[1] <= box[3] + tol


def test_stitch_separate_session():
    cfg = Config(sim_time_scale=0)
    base, _ = lattice_scans(cfg, False, y_range=(0.3, 3.3))  # living room only
    truth = Pose(3.0, 1.0, 1.1)
    session, _ = lattice_scans(cfg, True, frame=truth)  # whole house, own frame, furniture moved
    found = align_session(cfg, build_grid(cfg, base), session)
    assert found is not None
    t, overlap = found
    assert overlap > 0.6
    assert t.distance_to(truth.x, truth.y) < 0.08
    assert abs(math.remainder(t.theta - truth.theta, 2 * math.pi)) < math.radians(1)
    # A bare rectangular room fits itself turned 180 degrees just as well: refuses rather than guessing.
    room = np.array(_rect(0, 0, 4.0, 3.0), dtype=float)
    a, _ = lattice_scans(cfg, False, (0.4, 3.7), (0.4, 2.7), segs=room)
    b, _ = lattice_scans(cfg, False, (0.4, 3.7), (0.4, 2.7), frame=Pose(1, 0.5, 0.4), segs=room)
    assert align_session(cfg, build_grid(cfg, a), b) is None

def test_detect_moved_and_added_furniture():
    cfg = Config(sim_time_scale=0)
    before, robot = lattice_scans(cfg, False)
    after, _ = lattice_scans(cfg, True)
    gb, ga = build_grid(cfg, before), build_grid(cfg, after)
    changes = detect_changes(gb, ga)
    kinds = sorted(c.kind for c in changes)
    assert kinds == ["added", "moved"], kinds
    moved = describe(next(c for c in changes if c.kind == "moved"), gb)
    assert inside((moved["x"], moved["y"]), world_box_in_map(robot, SOFA))
    assert inside((moved["to"]["x"], moved["to"]["y"]), world_box_in_map(robot, SOFA_MOVED))
    added = describe(next(c for c in changes if c.kind == "added"), gb)
    assert inside((added["x"], added["y"]), world_box_in_map(robot, CHAIR))
    # Nothing changes between two scans of the same house.
    again, _ = lattice_scans(cfg, False, x_range=(0.45, 9.0), y_range=(0.45, 6.0))
    assert detect_changes(gb, build_grid(cfg, again)) == []

    # Composing: accepting the move clears the old sofa spot; rejecting keeps it.
    ids = np.array([0] * len(before) + [1] * len(after))

    def sofa(accept):
        sess = [{"id": 0}, {"id": 1, "changes": [change_record(n, c, accept) for n, c in enumerate(changes)]}]
        return occupied_at(compose(cfg, before + after, ids, sess), moved["points"])

    # (What stays is the wall face that was hidden behind the sofa.)
    assert sofa(False) > 80 and sofa(True) < sofa(False) / 3

    # The floor plan of the result: the sofa in its new place is furniture (hidden, ends too), and
    # the chair is drawn as a free-standing outline, not as walls.
    sess = [{"id": 0}, {"id": 1, "changes": [change_record(n, c) for n, c in enumerate(changes)]}]
    fp = extract(compose(cfg, before + after, ids, sess), [])
    for box, kind in ((SOFA_MOVED, "furniture"), (CHAIR, "fixture")):
        x0, y0, x1, y1 = world_box_in_map(robot, box)
        inside_box = [f for f in fp.faces if f.length > 0.3 and all(
            x0 - 0.1 <= x <= x1 + 0.1 and y0 - 0.1 <= y <= y1 + 0.1 for x, y in (f.a, f.b))]
        assert inside_box and {f.kind for f in inside_box} == {kind}, [(f.kind, f.a, f.b) for f in inside_box]


async def wait(svc, timeout=900):
    t0 = time.time()
    while svc.busy:
        assert time.time() - t0 < timeout, "mission timed out"
        await asyncio.sleep(0.05)
    return svc.mission


@pytest.fixture
def mapped(tmp_path):
    """A saved map of the original house (from lattice scans), as a finished mapping run leaves it."""
    cfg = Config(sim_time_scale=0, data_dir=str(tmp_path))
    kfs, _ = lattice_scans(cfg, False, step=0.7)
    store = Store(tmp_path)
    store.save_scans(kfs, [0] * len(kfs))
    store.save_grid(build_grid(cfg, kfs))
    return cfg, store


async def test_rescan_then_review(mapped):
    cfg, store = mapped
    robot = SimRobot(cfg, segments=house(True))  # someone moved the sofa and added a chair
    svc = Service(cfg, robot=robot, store=store)
    svc.status = await robot.status()
    assert store.sessions()[0]["status"] == "applied"  # maps from before sessions get one

    svc.start_mission("rescan")
    m = await wait(svc)
    assert m.status == "done", m.message
    assert "review" in m.message, m.message
    assert robot.pose.distance_to(robot.dock.x, robot.dock.y) < 0.01
    review = svc.review()
    kinds = sorted(c["kind"] for c in review["changes"])
    assert "moved" in kinds or ("removed" in kinds and "added" in kinds), kinds
    assert svc.snapshot()["review"]["changes"] == len(review["changes"])

    # A second scan can't start until this one is reviewed.
    svc.start_mission("rescan")
    m = await wait(svc)
    assert m.status == "failed" and "review" in m.message

    sofa = world_box_in_map(robot, SOFA)
    old_spot = next(c for c in review["changes"] if c["kind"] in ("moved", "removed") and inside((c["x"], c["y"]), sofa))

    def sofa(grid):
        return occupied_at(grid, old_spot["points"])

    there = sofa(store.load_grid())  # not applied yet
    assert there > 40
    svc.review_decide(old_spot["id"], False)
    assert sofa(await svc.preview_grid()) > there * 0.8
    svc.review_decide(old_spot["id"], True)
    assert sofa(await svc.preview_grid()) < there / 3
    plan = await svc.floorplan(preview=True)
    assert plan.interior_area_m2 > 45

    await svc.apply_review()
    assert svc.review() is None and store.pending_session() is None
    assert sofa(store.load_grid()) < there / 3
    assert [s["status"] for s in store.sessions()] == ["applied", "applied"]
    assert set(store.load_scan_ids()) == {0, 1}


async def test_extend_off_dock_is_stitched(mapped):
    cfg, store = mapped
    robot = SimRobot(cfg)
    svc = Service(cfg, robot=robot, store=store)
    robot.pose = Pose(7.0, 4.0, 2.0)  # in the kitchen, far from where the map thinks it is
    svc.status = await robot.status()
    before = len(store.load_scans())
    svc.start_mission("explore", new_map=False, max_goals=3)
    m = await wait(svc)
    assert m.status == "done", m.message
    assert robot.pose.distance_to(robot.dock.x, robot.dock.y) < 0.01  # found its way home after stitching
    s = store.sessions()[-1]
    assert s["stitched"] and s["status"] == "pending"  # stitched runs are always reviewed
    assert "line up" in m.message
    # Stitched where it really was: the robot's scans of an unchanged house show no changes.
    assert svc.review()["changes"] == []
    assert len(store.load_scans()) > before
    await svc.apply_review()
    assert store.pending_session() is None

def test_review_api(mapped):
    cfg, store = mapped
    # Fake a pending session made of the rearranged house.
    kfs, ids = store.load_scans(), list(store.load_scan_ids())
    new, _ = lattice_scans(cfg, True, step=0.7)
    store.save_scans(kfs + new, ids + [1] * len(new))
    before = compose(cfg, kfs, np.zeros(len(kfs)), [{"id": 0}])
    changes = detect_changes(before, build_grid(cfg, new))
    store.save_sessions(store.sessions() + [{
        "id": 1, "kind": "rescan", "started": 0, "status": "pending", "scans": len(new), "stitched": False,
        "changes": [change_record(i, c) for i, c in enumerate(changes)],
    }])
    with TestClient(create_app(cfg)) as c:
        r = c.get("/api/review").json()
        assert len(r["changes"]) == len(changes) and r["session"]["kind"] == "rescan"
        assert all(ch["points"] for ch in r["changes"])
        assert c.get("/api/state").json()["review"]["changes"] == len(changes)
        assert c.post("/api/review/changes/0", json={"accept": False}).json() == {"id": 0, "accept": False}
        assert c.post("/api/review/changes/99", json={"accept": False}).status_code == 404
        assert c.get("/api/review").json()["changes"][0]["accept"] is False
        assert not c.get("/api/review/map").json()["empty"]
        assert c.get("/api/floorplan?preview=1").status_code == 200
        assert [s["status"] for s in c.get("/api/sessions").json()] == ["applied", "pending"]
        assert c.post("/api/review/discard").json() == {"ok": True}
        assert c.get("/api/review").json() is None
        assert c.post("/api/review/apply").status_code == 404
        assert set(Store(cfg.data_dir).load_scan_ids()) == {0}
