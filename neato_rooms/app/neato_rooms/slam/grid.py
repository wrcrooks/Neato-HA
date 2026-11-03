"""Log-odds occupancy grid."""

from __future__ import annotations

import base64
import io

import numpy as np

from ..geometry import Pose

L_FREE = -0.4
L_OCC = 0.9
L_MIN, L_MAX = -4.0, 4.0
OCC_THRESH = 0.6
FREE_THRESH = -0.4

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


class OccupancyGrid:
    def __init__(self, resolution: float = 0.05, size_m: float = 40.0):
        self.res = float(resolution)
        n = int(round(size_m / resolution))
        self.n = n
        self.origin = (-n * resolution / 2.0, -n * resolution / 2.0)  # world coords of cell (0,0) corner
        self.logodds = np.zeros((n, n), dtype=np.float32)  # [iy, ix]
        # Running sum / count of LIDAR endpoints per cell: gives wall positions far finer
        # than the cell size (used by the floorplan extractor).
        self.hit_sum = np.zeros((n, n, 2), dtype=np.float32)
        self.hit_cnt = np.zeros((n, n), dtype=np.float32)
        self.version = 0

    # -- Coordinates ------------------------------------------------------------

    def world_to_cell(self, xy: np.ndarray) -> np.ndarray:
        """(N,2) world -> (N,2) integer [ix, iy]."""
        return np.floor((xy - np.array(self.origin)) / self.res).astype(int)

    def world_to_cellf(self, xy: np.ndarray) -> np.ndarray:
        return (xy - np.array(self.origin)) / self.res - 0.5

    def cell_to_world(self, ix, iy):
        return self.origin[0] + (np.asarray(ix) + 0.5) * self.res, self.origin[1] + (np.asarray(iy) + 0.5) * self.res

    def in_bounds(self, cells: np.ndarray) -> np.ndarray:
        return (cells[:, 0] >= 0) & (cells[:, 0] < self.n) & (cells[:, 1] >= 0) & (cells[:, 1] < self.n)

    # -- Update -----------------------------------------------------------------

    def integrate(self, pose: Pose, pts_robot: np.ndarray, sensor_robot: tuple[float, float]) -> None:
        if len(pts_robot) == 0:
            return
        sensor = pose.transform_points(np.array([sensor_robot]))[0]
        ends = pose.transform_points(pts_robot)
        vec = ends - sensor
        length = np.linalg.norm(vec, axis=1)
        step = self.res * 0.5
        n_samples = np.maximum((length - self.res) / step, 0).astype(int)
        max_n = int(n_samples.max()) if len(n_samples) else 0
        if max_n > 0:
            t = np.arange(max_n) * step  # (S,)
            unit = vec / np.maximum(length, 1e-9)[:, None]
            samples = sensor[None, None, :] + unit[:, None, :] * t[None, :, None]  # (M,S,2)
            mask = np.arange(max_n)[None, :] < n_samples[:, None]
            free_cells = self.world_to_cell(samples[mask])
            free_cells = free_cells[self.in_bounds(free_cells)]
            if len(free_cells):
                flat = np.unique(free_cells[:, 1] * self.n + free_cells[:, 0])
                self.logodds.flat[flat] += L_FREE
        hit = self.world_to_cell(ends)
        ok = self.in_bounds(hit)
        hit = hit[ok]
        if len(hit):
            np.add.at(self.hit_sum, (hit[:, 1], hit[:, 0]), ends[ok].astype(np.float32))
            np.add.at(self.hit_cnt, (hit[:, 1], hit[:, 0]), 1.0)
            flat = np.unique(hit[:, 1] * self.n + hit[:, 0])
            self.logodds.flat[flat] += L_OCC
        np.clip(self.logodds, L_MIN, L_MAX, out=self.logodds)
        self.version += 1

    # -- Views ------------------------------------------------------------------

    def classes(self) -> np.ndarray:
        out = np.full(self.logodds.shape, UNKNOWN, dtype=np.uint8)
        out[self.logodds < FREE_THRESH] = FREE
        out[self.logodds > OCC_THRESH] = OCCUPIED
        return out

    def occupied(self) -> np.ndarray:
        return self.logodds > OCC_THRESH

    def free(self) -> np.ndarray:
        return self.logodds < FREE_THRESH

    def known_bbox(self, pad: int = 4) -> tuple[int, int, int, int] | None:
        known = np.abs(self.logodds) > 0.05
        if not known.any():
            return None
        ys, xs = np.nonzero(known.any(axis=1))[0], np.nonzero(known.any(axis=0))[0]
        return (
            max(int(xs[0]) - pad, 0),
            max(int(ys[0]) - pad, 0),
            min(int(xs[-1]) + pad + 1, self.n),
            min(int(ys[-1]) + pad + 1, self.n),
        )

    def export(self) -> dict:
        """Compact JSON-able view for the PWA: cropped classes, base64 uint8, row 0 = lowest y."""
        bbox = self.known_bbox()
        if bbox is None:
            return {"empty": True, "resolution": self.res, "version": self.version}
        x0, y0, x1, y1 = bbox
        data = self.classes()[y0:y1, x0:x1]
        ox, oy = self.origin[0] + x0 * self.res, self.origin[1] + y0 * self.res
        return {
            "empty": False,
            "resolution": self.res,
            "width": x1 - x0,
            "height": y1 - y0,
            "origin": [ox, oy],
            "data": base64.b64encode(data.tobytes()).decode(),
            "version": self.version,
        }

    # -- Persistence --------------------------------------------------------------

    def to_bytes(self) -> bytes:
        buf = io.BytesIO()
        np.savez_compressed(buf, logodds=self.logodds, res=self.res, origin=np.array(self.origin),
                            hit_sum=self.hit_sum, hit_cnt=self.hit_cnt)
        return buf.getvalue()

    @classmethod
    def from_bytes(cls, raw: bytes) -> "OccupancyGrid":
        z = np.load(io.BytesIO(raw))
        g = cls.__new__(cls)
        g.logodds = z["logodds"].astype(np.float32)
        g.res = float(z["res"])
        g.n = g.logodds.shape[0]
        g.origin = tuple(float(v) for v in z["origin"])
        if "hit_cnt" in z:
            g.hit_sum = z["hit_sum"].astype(np.float32)
            g.hit_cnt = z["hit_cnt"].astype(np.float32)
        else:  # maps saved before hit statistics existed
            g.hit_sum = np.zeros((g.n, g.n, 2), dtype=np.float32)
            g.hit_cnt = np.zeros((g.n, g.n), dtype=np.float32)
        g.version = 1
        return g

    def hit_points(self, mask: np.ndarray) -> np.ndarray:
        """(N,2) mean endpoint position of each masked cell (cell centre if it has no hits)."""
        iy, ix = np.nonzero(mask)
        cnt = self.hit_cnt[iy, ix]
        cx, cy = self.cell_to_world(ix, iy)
        out = np.stack([cx, cy], axis=1)
        has = cnt > 0
        out[has] = self.hit_sum[iy[has], ix[has]] / cnt[has, None]
        return out

    def copy(self) -> "OccupancyGrid":
        return OccupancyGrid.from_bytes(self.to_bytes())
