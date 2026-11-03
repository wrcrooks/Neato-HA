"""End-to-end against the simulator: explore -> define room -> clean it -> dock."""

import math
import time

import numpy as np

from neato_rooms.config import Config
from neato_rooms.geometry import point_in_polygon
from neato_rooms.robot.sim import SimRobot
from neato_rooms.service import Service
from neato_rooms.store import Store


def world_poly_to_map(robot, poly):
    out = []
    for x, y in poly:
        p = robot.world_to_map(type(robot.pose)(x, y, 0))
        out.append([p.x, p.y])
    return out


async def wait(svc, timeout=600):
    t0 = time.time()
    while svc.busy:
        assert time.time() - t0 < timeout, "mission timed out"
        await __import__("asyncio").sleep(0.05)
    return svc.mission


async def test_explore_then_clean_kitchen(tmp_path):
    cfg = Config(sim_time_scale=0, data_dir=str(tmp_path))
    robot = SimRobot(cfg)
    svc = Service(cfg, robot=robot, store=Store(tmp_path))
    svc.status = await robot.status()

    t0 = time.time()
    svc.start_mission("explore")
    m = await wait(svc)
    print("explore:", m.status, m.message, f"{time.time() - t0:.1f}s, scans={len(svc.mapper.trajectory)}")
    assert m.status == "done", m.message
    assert svc.store.has_map()
    assert robot.pose.distance_to(robot.dock.x, robot.dock.y) < 0.01  # docked

    # Map must cover all rooms: free cells at the centre of each room.
    grid = svc.store.load_grid()
    free = grid.free()
    for wx, wy, name in [(3.5, 1.5, "living"), (3.0, 4.8, "bedroom"), (4.9, 4.8, "hall"), (7.2, 3.9, "kitchen")]:
        p = robot.world_to_map(type(robot.pose)(wx, wy, 0))
        c = grid.world_to_cell(np.array([[p.x, p.y]]))[0]
        assert free[c[1] - 1 : c[1] + 2, c[0] - 1 : c[0] + 2].any(), f"{name} not mapped"

    # Kitchen: x 5.4-9, y 3.4-6 in the world frame.
    kitchen_w = [[5.45, 3.45], [8.95, 3.45], [8.95, 5.95], [5.45, 5.95]]
    area = svc.store.add_area({"name": "Kitchen", "kind": "room", "polygon": world_poly_to_map(robot, kitchen_w)})

    svc.status = await robot.status()
    robot.swept.clear()
    t0 = time.time()
    svc.start_mission("clean", area_ids=[area["id"]])
    m = await wait(svc)
    print("clean:", m.status, m.message, f"{time.time() - t0:.1f}s, swept pts={len(robot.swept)}")
    assert m.status == "done", m.message
    assert robot.pose.distance_to(robot.dock.x, robot.dock.y) < 0.01

    # Coverage: fraction of reachable kitchen floor within 0.14 m of a swept position.
    xs, ys = np.meshgrid(np.arange(5.4, 9.0, 0.05) + 0.025, np.arange(3.4, 6.0, 0.05) + 0.025)
    reachable = np.ones_like(xs, dtype=bool)
    from neato_rooms.robot.sim import _seg_dist
    for idx in np.ndindex(xs.shape):
        reachable[idx] = _seg_dist(xs[idx], ys[idx], robot.segments) > cfg.robot_radius_m + 0.06
    sw = np.array(robot.swept)
    cells = np.stack([xs[reachable], ys[reachable]], 1)
    d = np.min(np.linalg.norm(cells[:, None, :] - sw[None, ::3, :], axis=2), axis=1)
    coverage = float((d < 0.14).mean())
    print(f"kitchen coverage {coverage:.1%}")
    assert coverage > 0.9
    # Vacuum must not run outside the kitchen (small tolerance for the doorway).
    inside = point_in_polygon(sw[:, 0], sw[:, 1], [[5.2, 3.2], [9, 3.2], [9, 6], [5.2, 6]])
    assert inside.mean() > 0.97
