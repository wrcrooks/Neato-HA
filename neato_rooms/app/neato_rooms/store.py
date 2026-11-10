"""Persistence for the map and the user's rooms / zones / no-go areas."""

from __future__ import annotations

import io
import json
import os
import time
import uuid
from pathlib import Path

import numpy as np

from .geometry import Pose
from .slam.grid import OccupancyGrid

AREA_KINDS = ("room", "zone", "nogo")
UNITS = ("imperial", "metric")
PAPERS = ("auto", "letter", "a4")
FLOORPLAN_DEFAULTS = {"rotation": None, "units": "imperial", "title": "Floor Plan", "paper": "auto", "overrides": []}
PALETTE = ["#4e79a7", "#f28e2b", "#59a14f", "#b07aa1", "#edc948", "#76b7b2", "#ff9da7", "#9c755f"]


def _atomic_write(path: Path, data: bytes) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


class Store:
    def __init__(self, data_dir: str | Path, map_name: str = "home"):
        self.dir = Path(data_dir) / "maps" / map_name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.areas: list[dict] = self._load_areas()

    # -- Map ---------------------------------------------------------------------

    @property
    def grid_path(self) -> Path:
        return self.dir / "grid.npz"

    def has_map(self) -> bool:
        return self.grid_path.is_file()

    def load_grid(self) -> OccupancyGrid | None:
        return OccupancyGrid.from_bytes(self.grid_path.read_bytes()) if self.has_map() else None

    def save_grid(self, grid: OccupancyGrid) -> None:
        _atomic_write(self.grid_path, grid.to_bytes())
        meta = {"saved": time.time(), "resolution": grid.res}
        _atomic_write(self.dir / "meta.json", json.dumps(meta).encode())

    def delete_map(self) -> None:
        for name in ("grid.npz", "meta.json", "scans.npz", "sessions.json"):
            (self.dir / name).unlink(missing_ok=True)
        # Alignment and hidden walls belong to the old map; keep units/title/paper.
        self.save_floorplan_settings({**self.floorplan_settings(), "rotation": None, "overrides": []})

    # -- Scans (keyframes used to refine the map), tagged with their mapping session ------

    def save_scans(self, keyframes: list[tuple[Pose, np.ndarray]], session_ids=None) -> None:
        poses = np.array([[p.x, p.y, p.theta] for p, _ in keyframes], dtype=np.float64).reshape(-1, 3)
        counts = np.array([len(pts) for _, pts in keyframes], dtype=np.int32)
        pts = np.concatenate([pts for _, pts in keyframes]) if keyframes else np.zeros((0, 2))
        ids = np.zeros(len(keyframes), dtype=np.int32) if session_ids is None else np.asarray(session_ids, dtype=np.int32)
        if len(ids) != len(keyframes):
            raise ValueError("one session id per scan")
        buf = io.BytesIO()
        np.savez_compressed(buf, poses=poses, counts=counts, points=pts.astype(np.float32), session=ids)
        _atomic_write(self.dir / "scans.npz", buf.getvalue())

    def _scans_npz(self):
        p = self.dir / "scans.npz"
        return np.load(io.BytesIO(p.read_bytes())) if p.is_file() else None

    def load_scans(self) -> list[tuple[Pose, np.ndarray]]:
        z = self._scans_npz()
        if z is None:
            return []
        chunks = np.split(z["points"], np.cumsum(z["counts"])[:-1]) if len(z["counts"]) else []
        return [(Pose(*map(float, pose)), pts) for pose, pts in zip(z["poses"], chunks)]

    def load_scan_ids(self) -> np.ndarray:
        """Session id of each saved scan (all 0 for maps saved before sessions existed)."""
        z = self._scans_npz()
        if z is None:
            return np.zeros(0, dtype=np.int32)
        return z["session"] if "session" in z else np.zeros(len(z["counts"]), dtype=np.int32)

    # -- Mapping sessions ---------------------------------------------------------------

    def sessions(self) -> list[dict]:
        """Mapping runs on this map, oldest first: {id, kind, started, status, scans, stitched, changes}.

        status is "applied" (part of the map) or "pending" (waiting for its changes to be reviewed).
        """
        p = self.dir / "sessions.json"
        if p.is_file():
            return json.loads(p.read_text())
        if (self.dir / "scans.npz").is_file():  # map saved before sessions existed
            n = len(self.load_scan_ids())
            return [{"id": 0, "kind": "map", "started": self.meta().get("saved", 0), "status": "applied", "scans": n,
                     "stitched": False, "changes": []}]
        return []

    def save_sessions(self, sessions: list[dict]) -> None:
        _atomic_write(self.dir / "sessions.json", json.dumps(sessions).encode())

    def pending_session(self) -> dict | None:
        return next((s for s in self.sessions() if s["status"] == "pending"), None)

    # -- Floor plan settings ------------------------------------------------------------

    def floorplan_settings(self) -> dict:
        p = self.dir / "floorplan.json"
        saved = json.loads(p.read_text()) if p.is_file() else {}
        return {**FLOORPLAN_DEFAULTS, **saved}

    def save_floorplan_settings(self, settings: dict) -> dict:
        s = {**FLOORPLAN_DEFAULTS, **{k: v for k, v in settings.items() if k in FLOORPLAN_DEFAULTS}}
        if s["units"] not in UNITS:
            raise ValueError(f"units must be one of {UNITS}")
        if s["paper"] not in PAPERS:
            raise ValueError(f"paper must be one of {PAPERS}")
        s["title"] = str(s["title"] or "").strip()[:80]
        s["rotation"] = None if s["rotation"] is None else float(s["rotation"])
        _atomic_write(self.dir / "floorplan.json", json.dumps(s, indent=1).encode())
        return s

    def meta(self) -> dict:
        p = self.dir / "meta.json"
        return json.loads(p.read_text()) if p.is_file() else {}

    # -- Areas -------------------------------------------------------------------

    def _load_areas(self) -> list[dict]:
        p = self.dir / "areas.json"
        return json.loads(p.read_text()) if p.is_file() else []

    def _save_areas(self) -> None:
        _atomic_write(self.dir / "areas.json", json.dumps(self.areas, indent=1).encode())

    @staticmethod
    def validate(area: dict) -> dict:
        kind = area.get("kind", "room")
        if kind not in AREA_KINDS:
            raise ValueError(f"kind must be one of {AREA_KINDS}")
        poly = area.get("polygon") or []
        if len(poly) < 3:
            raise ValueError("polygon needs at least 3 points")
        poly = [[float(p[0]), float(p[1])] for p in poly]
        name = str(area.get("name") or kind.title()).strip()[:60]
        return {"kind": kind, "polygon": poly, "name": name, "color": area.get("color")}

    def add_area(self, area: dict) -> dict:
        a = self.validate(area)
        a["id"] = uuid.uuid4().hex[:8]
        a["color"] = a["color"] or PALETTE[len(self.areas) % len(PALETTE)]
        self.areas.append(a)
        self._save_areas()
        return a

    def update_area(self, area_id: str, patch: dict) -> dict:
        for i, a in enumerate(self.areas):
            if a["id"] == area_id:
                merged = self.validate({**a, **patch})
                merged["id"] = area_id
                merged["color"] = merged["color"] or a["color"]
                self.areas[i] = merged
                self._save_areas()
                return merged
        raise KeyError(area_id)

    def delete_area(self, area_id: str) -> None:
        before = len(self.areas)
        self.areas = [a for a in self.areas if a["id"] != area_id]
        if len(self.areas) == before:
            raise KeyError(area_id)
        self._save_areas()

    def get_area(self, area_id: str) -> dict:
        for a in self.areas:
            if a["id"] == area_id:
                return a
        raise KeyError(area_id)

    def nogo_polygons(self) -> list[list[list[float]]]:
        return [a["polygon"] for a in self.areas if a["kind"] == "nogo"]
