"""Floor-plan extraction on the simulated house, rendering, and the plan API."""

import asyncio
import math
import zlib

import numpy as np
import pytest
from fastapi.testclient import TestClient

from neato_rooms.api import create_app
from neato_rooms.config import Config
from neato_rooms.floorplan import extract, layout, to_pdf, to_svg
from neato_rooms.geometry import Pose
from neato_rooms.robot.sim import SimRobot, _seg_dist
from neato_rooms.slam.grid import OccupancyGrid
from neato_rooms.store import Store

DOCK_DEG = 20.0
# World-frame room rectangles (drawn a little inside the walls, as a user would) and the
# true interior sizes from the sim house.
ROOMS = {
    "Living room": ((0.2, 0.2), (8.6, 3.1), (9.0, 3.34)),
    "Bedroom": ((0.2, 3.7), (4.1, 5.8), (4.34, 2.54)),
    "Kitchen": ((5.7, 3.7), (8.8, 5.8), (3.54, 2.54)),
    "Hall": ((4.55, 3.7), (5.25, 5.8), (0.88, 2.54)),
}


FURNITURE = [(0.0, 2.2, 2.0, 3.0), (6.8, 4.4, 7.8, 5.0), (0.0, 4.4, 2.0, 6.0)]  # sofa, island, bed


def _in_furniture(x, y) -> bool:
    return any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in FURNITURE)


def _scan_house(cfg: Config) -> tuple[OccupancyGrid, SimRobot]:
    """Map the sim house from known poses on a lattice (no SLAM error), dock rotated."""
    robot = SimRobot(cfg, seed=3)
    d = robot.dock
    robot.dock = Pose(d.x, d.y, math.radians(DOCK_DEG))
    robot._manual = True
    grid = OccupancyGrid(cfg.map_resolution_m, cfg.map_size_m)
    sensor = (cfg.lidar_offset_x_mm / 1000.0, 0.0)
    for x in np.arange(0.3, 9.0, 0.35):
        for y in np.arange(0.3, 6.0, 0.35):
            if _seg_dist(x, y, robot.segments) < cfg.robot_radius_m + 0.05 or _in_furniture(x, y):
                continue
            for theta in (0.0, math.pi / 2):
                robot.pose = Pose(float(x), float(y), theta)
                scan = asyncio.run(robot.scan())
                grid.integrate(robot.world_to_map(robot.pose), scan.to_points(cfg), sensor)
    return grid, robot


@pytest.fixture(scope="module")
def house():
    cfg = Config(sim_time_scale=0)
    grid, robot = _scan_house(cfg)

    def to_map(x, y):
        p = robot.world_to_map(Pose(x, y, 0))
        return [p.x, p.y]

    areas = [
        {"id": n, "name": n, "kind": "room", "polygon": [to_map(a, b), to_map(c, b), to_map(c, e), to_map(a, e)]}
        for n, ((a, b), (c, e), _) in ROOMS.items()
    ]
    return grid, areas


def test_extract_matches_house(house):
    grid, areas = house
    fp = extract(grid, areas)
    # Aligned to the walls: the map is rotated by the dock heading.
    assert abs(math.degrees(fp.rotation) + DOCK_DEG) < 0.5
    rooms = {r.name: r for r in fp.rooms}
    for name, (_, _, (w, dp)) in ROOMS.items():
        r = rooms[name]
        assert r.width_m == pytest.approx(w, abs=0.08), name
        assert r.depth_m == pytest.approx(dp, abs=0.08), name
    assert len(fp.doors) == 2
    for d in fp.doors:
        assert d.width == pytest.approx(0.8, abs=0.08)
    assert fp.interior_area_m2 == pytest.approx(53.8, abs=1.5)
    # Sofa and bed are hidden; the kitchen island is a fixture; the chamfer is found.
    assert sum(f.kind == "furniture" for f in fp.faces) >= 3
    assert all(not f.visible for f in fp.faces if f.kind == "furniture")
    assert sum(f.kind == "fixture" for f in fp.faces) >= 2
    assert any(abs(math.degrees(f.angle) - 45) < 1 for f in fp.walls())
    assert sum(f.kind == "exterior" for f in fp.faces) >= 4


def test_render_svg_pdf(house):
    grid, areas = house
    fp = extract(grid, areas)
    for units in ("imperial", "metric"):
        page = layout(fp, units, "Test House")
        svg = to_svg(page)
        assert svg.startswith("<svg") or svg.startswith("<?xml")
        assert "BEDROOM" in svg and "Test House" in svg
        pdf = to_pdf(page, "Test House")
        assert pdf.startswith(b"%PDF-") and pdf.rstrip().endswith(b"%%EOF")
        # The page content stream inflates and contains the title.
        start = pdf.index(b"stream\n") + 7
        end = pdf.index(b"endstream", start)
        assert b"Test House" in zlib.decompress(pdf[start:end].rstrip(b"\n"))
    assert "sq ft" in to_svg(layout(fp, "imperial", ""))
    assert "m²" in to_svg(layout(fp, "metric", ""))


def test_floorplan_api(house, tmp_path):
    grid, areas = house
    store = Store(tmp_path)
    store.save_grid(grid)
    for a in areas:
        store.add_area(a)
    cfg = Config(sim_time_scale=0.02, data_dir=str(tmp_path))
    with TestClient(create_app(cfg)) as c:
        fp = c.get("/api/floorplan").json()
        assert fp["settings"]["rotation"] is None
        assert {r["name"] for r in fp["rooms"]} == set(ROOMS)
        assert all(r["dims"] for r in fp["rooms"]) and "sq ft" in fp["area_text"]

        # Rotate by 90 degrees, then align to a 45-degree wall, then back to automatic.
        s = c.post("/api/floorplan/align", json={"mode": "rotate", "delta_deg": 90}).json()
        assert s["rotation"] == pytest.approx((fp["rotation"] + math.pi / 2) % (2 * math.pi))
        diag = next(f for f in c.get("/api/floorplan").json()["faces"]
                    if f["visible"] and f["kind"] in ("wall", "exterior") and f["length"] > 0.5
                    and abs(math.degrees(f["angle"]) % 90 - 45) < 1)
        c.post("/api/floorplan/align", json={"mode": "face", "face_id": diag["id"]})
        aligned = c.get("/api/floorplan").json()
        assert abs((aligned["rotation"] - fp["rotation"]) % (math.pi / 2) - math.pi / 4) < 0.02
        assert c.post("/api/floorplan/align", json={"mode": "face", "face_id": 999}).status_code == 404
        c.post("/api/floorplan/align", json={"mode": "auto"})
        assert c.get("/api/floorplan").json()["rotation"] == pytest.approx(fp["rotation"])

        # Hiding a wall hides only that face, and survives the toggle back.
        wall = next(f for f in fp["faces"] if f["visible"] and f["kind"] == "wall")
        c.post("/api/floorplan/toggle", json={"face_id": wall["id"]})
        after = c.get("/api/floorplan").json()["faces"]
        assert sum(not f["visible"] for f in after) == sum(not f["visible"] for f in fp["faces"]) + 1
        c.post("/api/floorplan/toggle", json={"face_id": wall["id"]})
        assert sum(not f["visible"] for f in c.get("/api/floorplan").json()["faces"]) == sum(
            not f["visible"] for f in fp["faces"])

        assert c.put("/api/floorplan/settings", json={"units": "metric", "title": "Flat 3"}).json()["units"] == "metric"
        assert c.put("/api/floorplan/settings", json={"units": "cubits"}).status_code == 400
        r = c.get("/api/floorplan.svg?download=1")
        assert r.headers["content-type"].startswith("image/svg+xml")
        assert 'filename="Flat_3.svg"' in r.headers["content-disposition"]
        assert "Flat 3" in r.text and "m²" in r.text
        r = c.get("/api/floorplan.pdf")
        assert r.content.startswith(b"%PDF-") and "content-disposition" not in r.headers

        # Deleting the map resets alignment/overrides but keeps the title.
        assert c.delete("/api/map").status_code == 200
        assert c.get("/api/floorplan").status_code == 404
        assert c.get("/api/floorplan/settings").json()["title"] == "Flat 3"


def test_scans_roundtrip(tmp_path):
    store = Store(tmp_path)
    kfs = [(Pose(0.1 * i, 0.2, 0.3), np.random.rand(10 + i, 2).astype(np.float32)) for i in range(4)]
    store.save_scans(kfs)
    back = store.load_scans()
    assert len(back) == 4
    for (p, pts), (q, qts) in zip(kfs, back):
        assert (p.x, p.y, p.theta) == pytest.approx((q.x, q.y, q.theta))
        np.testing.assert_allclose(pts, qts)
