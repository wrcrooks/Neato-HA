"""Correlative scan-to-map matching (brute-force search over x, y, theta)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..geometry import Pose, wrap_angle
from .grid import OccupancyGrid


@dataclass
class MatchResult:
    pose: Pose
    score: float  # mean likelihood of scan endpoints, 0..1


def _blur(a: np.ndarray, sigma: float) -> np.ndarray:
    r = max(1, int(math.ceil(3 * sigma)))
    k = np.exp(-(np.arange(-r, r + 1) ** 2) / (2 * sigma * sigma)).astype(np.float32)
    out = a
    for axis in (0, 1):
        pad = [(0, 0), (0, 0)]
        pad[axis] = (r, r)
        p = np.pad(out, pad)
        acc = np.zeros_like(out)
        n = out.shape[axis]
        for i, w in enumerate(k):
            sl = [slice(None), slice(None)]
            sl[axis] = slice(i, i + n)
            acc += w * p[tuple(sl)]
        out = acc
    return out


class LikelihoodField:
    """Blurred occupied-cell map around a point, values clipped to [0, 1]."""

    def __init__(self, grid: OccupancyGrid, center_xy: tuple[float, float], half_m: float, sigma_m: float = 0.06):
        cx, cy = grid.world_to_cell(np.array([center_xy]))[0]
        h = int(math.ceil(half_m / grid.res))
        self.x0, self.y0 = max(cx - h, 0), max(cy - h, 0)
        x1, y1 = min(cx + h, grid.n), min(cy + h, grid.n)
        occ = grid.occupied()[self.y0 : y1, self.x0 : x1].astype(np.float32)
        self.field = np.clip(_blur(occ, sigma_m / grid.res), 0, 1)
        self.grid = grid
        self.n_occupied = int(occ.sum())

    def _cellf(self, pts_w: np.ndarray) -> np.ndarray:
        c = self.grid.world_to_cellf(pts_w)
        c[:, 0] -= self.x0
        c[:, 1] -= self.y0
        return c

    def lookup_int(self, ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
        h, w = self.field.shape
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        out = np.zeros(ix.shape, dtype=np.float32)
        out[ok] = self.field[iy[ok], ix[ok]]
        return out

    def lookup_bilinear(self, fx: np.ndarray, fy: np.ndarray) -> np.ndarray:
        x0, y0 = np.floor(fx).astype(int), np.floor(fy).astype(int)
        ax, ay = fx - x0, fy - y0
        v00 = self.lookup_int(x0, y0)
        v10 = self.lookup_int(x0 + 1, y0)
        v01 = self.lookup_int(x0, y0 + 1)
        v11 = self.lookup_int(x0 + 1, y0 + 1)
        return (v00 * (1 - ax) + v10 * ax) * (1 - ay) + (v01 * (1 - ax) + v11 * ax) * ay


def _rotate(pts: np.ndarray, th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return pts @ np.array([[c, -s], [s, c]]).T


def match(
    grid: OccupancyGrid,
    pts_robot: np.ndarray,
    guess: Pose,
    lin_window_m: float = 0.25,
    ang_window_rad: float = math.radians(12),
    lin_step_cells: int = 1,
    ang_step_rad: float = math.radians(1.0),
    field: LikelihoodField | None = None,
    max_range_m: float = 5.0,
    prior_lin_m: float | None = None,
    prior_ang_rad: float | None = None,
    prior_weight: float = 0.05,
) -> MatchResult:
    """Search for the pose maximising scan/map agreement minus a Gaussian prior around `guess`.

    The prior matters: grid matching snaps scans to cell centres (up to res/2 bias), and over a
    short stationary step wheel odometry is more accurate than that.
    """
    if len(pts_robot) == 0:
        return MatchResult(guess, 0.0)
    if field is None:
        field = LikelihoodField(grid, (guess.x, guess.y), lin_window_m + max_range_m + 0.5)
    res = grid.res

    k = int(math.ceil(lin_window_m / res))
    offs = np.arange(-k, k + 1, lin_step_cells)
    ox, oy = np.meshgrid(offs, offs, indexing="xy")
    ox, oy = ox.ravel(), oy.ravel()

    n_ang = int(math.ceil(ang_window_rad / ang_step_rad))
    thetas = guess.theta + np.arange(-n_ang, n_ang + 1) * ang_step_rad

    sl = prior_lin_m if prior_lin_m else lin_window_m
    sa = prior_ang_rad if prior_ang_rad else ang_window_rad

    def prior(dx, dy, dth):
        return prior_weight * 0.5 * ((dx * dx + dy * dy) / (sl * sl) + (dth * dth) / (sa * sa))

    best = (-1e9, guess.x, guess.y, guess.theta)
    for th in thetas:
        pw = _rotate(pts_robot, th) + np.array([guess.x, guess.y])
        c = field._cellf(pw)
        bx, by = np.rint(c[:, 0]).astype(int), np.rint(c[:, 1]).astype(int)
        vals = field.lookup_int(bx[:, None] + ox[None, :], by[:, None] + oy[None, :])
        scores = vals.mean(axis=0) - prior(ox * res, oy * res, wrap_angle(th - guess.theta))
        i = int(np.argmax(scores))
        if scores[i] > best[0]:
            best = (float(scores[i]), guess.x + ox[i] * res, guess.y + oy[i] * res, float(th))

    # Refinement with bilinear interpolation around the coarse optimum.
    _, bx, by, bth = best
    fine = np.linspace(-lin_step_cells * res, lin_step_cells * res, 9)
    fx, fy = np.meshgrid(fine, fine, indexing="xy")
    fx, fy = fx.ravel(), fy.ravel()
    top = (-1e9, bx, by, bth)
    for th in bth + np.linspace(-ang_step_rad, ang_step_rad, 9):
        pw = _rotate(pts_robot, th)
        c = field._cellf(pw + np.array([bx, by]))
        vals = field.lookup_bilinear(c[:, 0:1] + fx[None, :] / res, c[:, 1:2] + fy[None, :] / res)
        raw = vals.mean(axis=0)
        scores = raw - prior(bx + fx - guess.x, by + fy - guess.y, wrap_angle(th - guess.theta))
        i = int(np.argmax(scores))
        if scores[i] > top[0]:
            top = (float(scores[i]), bx + fx[i], by + fy[i], float(th), float(raw[i]))
    return MatchResult(Pose(top[1], top[2], wrap_angle(top[3])), top[4])
