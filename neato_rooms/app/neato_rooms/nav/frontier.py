"""Frontier-based exploration goal selection."""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from .astar import dijkstra
from .costmap import CostMap, dilate


def frontier_mask(cm: CostMap) -> np.ndarray:
    """Free cells with at least one 4-neighbour unknown."""
    u = cm.unknown
    nb = np.zeros_like(u)
    nb[1:, :] |= u[:-1, :]
    nb[:-1, :] |= u[1:, :]
    nb[:, 1:] |= u[:, :-1]
    nb[:, :-1] |= u[:, 1:]
    return cm.free & nb


def clusters(mask: np.ndarray, min_size: int) -> list[list[tuple[int, int]]]:
    h, w = mask.shape
    seen = np.zeros_like(mask)
    out = []
    for y, x in zip(*np.nonzero(mask)):
        if seen[y, x]:
            continue
        comp = []
        q = deque([(x, y)])
        seen[y, x] = True
        while q:
            cx, cy = q.popleft()
            comp.append((cx, cy))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        q.append((nx, ny))
        if len(comp) >= min_size:
            out.append(comp)
    return out


def choose_goal(
    cm: CostMap,
    robot_xy: tuple[float, float],
    tried: list[tuple[float, float]],
    min_cluster_m: float = 0.3,
    reach_m: float = 0.45,
) -> tuple[float, float] | None:
    """Nearest reachable spot from which a sizeable frontier can be observed."""
    min_size = max(3, int(min_cluster_m / cm.res))
    big = np.zeros(cm.shape, dtype=bool)
    for comp in clusters(frontier_mask(cm), min_size):
        xs, ys = zip(*comp)
        big[list(ys), list(xs)] = True
    if not big.any():
        return None
    approach = dilate(big, int(math.ceil(reach_m / cm.res))) & cm.traversable
    start = cm.nearest_traversable(*cm.to_cell(*robot_xy), max_r=8)
    if start is None:
        return None
    dist = dijkstra(cm, start)
    cand = approach & np.isfinite(dist)
    # Don't go back to places that were tried and didn't resolve the frontier.
    for tx, ty in tried:
        cx, cy = cm.to_cell(tx, ty)
        r = int(0.35 / cm.res)
        y0, y1 = max(cy - r, 0), min(cy + r + 1, cm.shape[0])
        x0, x1 = max(cx - r, 0), min(cx + r + 1, cm.shape[1])
        cand[y0:y1, x0:x1] = False
    if not cand.any():
        return None
    d = np.where(cand, dist, np.inf)
    # Ignore trivially-close goals (we are already observing those frontiers).
    d[d < 0.25 / cm.res] = np.inf
    if not np.isfinite(d).any():
        d = np.where(cand, dist, np.inf)
    iy, ix = np.unravel_index(int(np.argmin(d)), d.shape)
    return cm.to_world(ix, iy)
