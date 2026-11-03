"""Boustrophedon (lawn-mower) coverage of an area polygon."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .costmap import CostMap, rasterize_polygon


@dataclass
class Lane:
    start: tuple[float, float]
    end: tuple[float, float]

    @property
    def length(self) -> float:
        return math.dist(self.start, self.end)

    def reversed(self) -> "Lane":
        return Lane(self.end, self.start)


def area_mask(cm: CostMap, polygon: list[list[float]]) -> np.ndarray:
    """Cells inside the polygon where the robot centre can be."""
    return rasterize_polygon(polygon, cm.origin[0], cm.origin[1], cm.res, cm.shape) & cm.traversable


def plan_lanes(
    cm: CostMap,
    polygon: list[list[float]],
    spacing_m: float,
    start_xy: tuple[float, float] | None = None,
    min_lane_m: float = 0.1,
) -> list[Lane]:
    mask = area_mask(cm, polygon)
    if not mask.any():
        return []
    ys, xs = np.nonzero(mask)
    # Lanes run along the longer side of the area -> fewer turns.
    horizontal = (xs.max() - xs.min()) >= (ys.max() - ys.min())
    if not horizontal:
        mask = mask.T
    lo, hi = (ys.min(), ys.max()) if horizontal else (xs.min(), xs.max())
    step = spacing_m / cm.res
    first = lo + min(step / 2, (hi - lo) / 2)
    rows = []
    r = first
    while r <= hi + 1e-6:
        rows.append(int(round(r)))
        r += step
    if rows and hi - rows[-1] > step * 0.4:
        rows.append(int(hi))

    raw: list[list[Lane]] = []
    min_cells = max(1, int(min_lane_m / cm.res))
    for row in rows:
        line = mask[row]
        lanes = []
        c = 0
        n = len(line)
        while c < n:
            if not line[c]:
                c += 1
                continue
            s = c
            while c < n and line[c]:
                c += 1
            e = c - 1
            if e - s + 1 >= min_cells:
                if horizontal:
                    a, b = cm.to_world(s, row), cm.to_world(e, row)
                else:
                    a, b = cm.to_world(row, s), cm.to_world(row, e)
                lanes.append(Lane(a, b))
        raw.append(lanes)

    # Order: sweep lanes row by row, alternating direction; when a row is split by an
    # obstacle pick the piece nearest to where we are.
    remaining = [ln for row in raw for ln in row]
    if not remaining:
        return []
    pos = start_xy or remaining[0].start
    ordered: list[Lane] = []
    while remaining:
        best_i, best_d, best_rev = 0, math.inf, False
        for i, ln in enumerate(remaining):
            for rev, p in ((False, ln.start), (True, ln.end)):
                d = math.dist(pos, p)
                if d < best_d:
                    best_i, best_d, best_rev = i, d, rev
        ln = remaining.pop(best_i)
        ln = ln.reversed() if best_rev else ln
        ordered.append(ln)
        pos = ln.end
    return ordered
