"""Grid search: A* to a goal, Dijkstra distance field, path shortcutting."""

from __future__ import annotations

import heapq
import math

import numpy as np

from .costmap import CostMap

SQRT2 = math.sqrt(2)
NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


def astar(cm: CostMap, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
    if not (cm.is_traversable(*start) and cm.is_traversable(*goal)):
        return None
    h, w = cm.shape
    trav = cm.traversable
    cost = cm.cost
    g = np.full((h, w), np.inf, dtype=np.float64)
    parent = np.full((h, w), -1, dtype=np.int64)
    sx, sy = start
    gx, gy = goal
    g[sy, sx] = 0.0
    openq = [(math.hypot(gx - sx, gy - sy), 0.0, sx, sy)]
    while openq:
        _, gc, x, y = heapq.heappop(openq)
        if (x, y) == goal:
            break
        if gc > g[y, x]:
            continue
        for dx, dy, step in NEIGHBOURS:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h) or not trav[ny, nx]:
                continue
            if dx and dy and not (trav[y, nx] and trav[ny, x]):
                continue  # no corner cutting
            ng = gc + step * cost[ny, nx]
            if ng < g[ny, nx]:
                g[ny, nx] = ng
                parent[ny, nx] = y * w + x
                heapq.heappush(openq, (ng + math.hypot(gx - nx, gy - ny), ng, nx, ny))
    if not np.isfinite(g[gy, gx]):
        return None
    path = [(gx, gy)]
    cur = gy * w + gx
    while cur != sy * w + sx:
        cur = int(parent.flat[cur])
        path.append((cur % w, cur // w))
    path.reverse()
    return path


def dijkstra(cm: CostMap, start: tuple[int, int]) -> np.ndarray:
    h, w = cm.shape
    trav = cm.traversable
    dist = np.full((h, w), np.inf)
    if not cm.is_traversable(*start):
        return dist
    dist[start[1], start[0]] = 0.0
    q = [(0.0, start[0], start[1])]
    while q:
        d, x, y = heapq.heappop(q)
        if d > dist[y, x]:
            continue
        for dx, dy, step in NEIGHBOURS:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and trav[ny, nx]:
                if dx and dy and not (trav[y, nx] and trav[ny, x]):
                    continue
                nd = d + step * cm.cost[ny, nx]
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    heapq.heappush(q, (nd, nx, ny))
    return dist


def shortcut(cm: CostMap, path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Greedy line-of-sight simplification -> sparse waypoints."""
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not cm.line_clear(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan_path(cm: CostMap, start_xy: tuple[float, float], goal_xy: tuple[float, float]) -> list[tuple[float, float]] | None:
    """World-coordinate waypoints from start to goal (start excluded), or None if unreachable."""
    s = cm.nearest_traversable(*cm.to_cell(*start_xy), max_r=8)
    gl = cm.nearest_traversable(*cm.to_cell(*goal_xy), max_r=6)
    if s is None or gl is None:
        return None
    cells = astar(cm, s, gl)
    if cells is None:
        return None
    pts = [cm.to_world(*c) for c in shortcut(cm, cells)]
    return pts[1:] if len(pts) > 1 else pts
