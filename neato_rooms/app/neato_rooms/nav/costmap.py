"""Traversability map derived from the occupancy grid, no-go areas and runtime obstacles."""

from __future__ import annotations

import math

import numpy as np

from ..geometry import point_in_polygon
from ..slam.grid import FREE_THRESH, OCC_THRESH, OccupancyGrid


def disk_offsets(r_cells: int) -> list[tuple[int, int]]:
    return [(dx, dy) for dx in range(-r_cells, r_cells + 1) for dy in range(-r_cells, r_cells + 1) if dx * dx + dy * dy <= r_cells * r_cells]


def dilate(mask: np.ndarray, r_cells: int) -> np.ndarray:
    if r_cells <= 0:
        return mask.copy()
    h, w = mask.shape
    p = np.pad(mask, r_cells)
    out = np.zeros_like(mask)
    for dx, dy in disk_offsets(r_cells):
        out |= p[r_cells + dy : r_cells + dy + h, r_cells + dx : r_cells + dx + w]
    return out


def rasterize_polygon(poly: list[list[float]], x0: float, y0: float, res: float, shape: tuple[int, int]) -> np.ndarray:
    h, w = shape
    xs = x0 + (np.arange(w) + 0.5) * res
    ys = y0 + (np.arange(h) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    return point_in_polygon(gx, gy, poly)


class CostMap:
    """Cropped to the known part of the grid. Cell (ix, iy) here = grid cell (ix + x0c, iy + y0c)."""

    def __init__(
        self,
        grid: OccupancyGrid,
        robot_radius_m: float,
        margin_m: float = 0.03,
        nogo: list[list[list[float]]] | None = None,
        obstacles: list[tuple[float, float]] | None = None,
        visited: list[tuple[float, float]] | None = None,
    ):
        self.res = grid.res
        bbox = grid.known_bbox(pad=8) or (grid.n // 2 - 10, grid.n // 2 - 10, grid.n // 2 + 10, grid.n // 2 + 10)
        self.x0c, self.y0c, x1, y1 = bbox
        self.origin = (grid.origin[0] + self.x0c * self.res, grid.origin[1] + self.y0c * self.res)
        lo = grid.logodds[self.y0c : y1, self.x0c : x1]
        occ = lo > OCC_THRESH
        self.free = lo < FREE_THRESH
        self.unknown = ~occ & ~self.free
        shape = occ.shape
        self.shape = shape
        hard = occ.copy()
        for poly in nogo or []:
            hard |= rasterize_polygon(poly, self.origin[0], self.origin[1], self.res, shape)
        for ox, oy in obstacles or []:
            c = self.to_cell(ox, oy)
            if self.inside(*c):
                hard[c[1], c[0]] = True
        # Cells the robot has physically occupied are free, whatever the LIDAR says.
        for vx, vy in visited or []:
            cx, cy = self.to_cell(vx, vy)
            for dx, dy in disk_offsets(2):
                ix, iy = cx + dx, cy + dy
                if self.inside(ix, iy) and not occ[iy, ix]:
                    self.free[iy, ix] = True
                    self.unknown[iy, ix] = False
        r = int(math.ceil((robot_radius_m + margin_m) / self.res))
        self.blocked = dilate(hard, r)
        self.traversable = self.free & ~self.blocked
        # Soft cost: prefer paths away from walls.
        near = dilate(hard, r + 3) & ~self.blocked
        self.cost = np.ones(shape, dtype=np.float32)
        self.cost[near] = 4.0

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        return int(math.floor((x - self.origin[0]) / self.res)), int(math.floor((y - self.origin[1]) / self.res))

    def to_world(self, ix: float, iy: float) -> tuple[float, float]:
        return self.origin[0] + (ix + 0.5) * self.res, self.origin[1] + (iy + 0.5) * self.res

    def inside(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.shape[1] and 0 <= iy < self.shape[0]

    def is_traversable(self, ix: int, iy: int) -> bool:
        return self.inside(ix, iy) and bool(self.traversable[iy, ix])

    def nearest_traversable(self, ix: int, iy: int, max_r: int = 10) -> tuple[int, int] | None:
        if self.is_traversable(ix, iy):
            return ix, iy
        best = None
        for r in range(1, max_r + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r) if abs(dx) != r else range(-r, r + 1):
                    if self.is_traversable(ix + dx, iy + dy):
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, ix + dx, iy + dy)
            if best:
                return best[1], best[2]
        return None

    def line_clear(self, a: tuple[int, int], b: tuple[int, int]) -> bool:
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
        xs = np.rint(np.linspace(a[0], b[0], n)).astype(int)
        ys = np.rint(np.linspace(a[1], b[1], n)).astype(int)
        if not all(self.inside(x, y) for x, y in ((xs.min(), ys.min()), (xs.max(), ys.max()))):
            return False
        return bool(self.traversable[ys, xs].all())
