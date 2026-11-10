"""Several mapping sessions on one map: stitching, change detection and the combined map.

Every mapping run (a new map, an extension, a re-scan) is a *session*: its keyframes are
kept separately, tagged with the session id. A run that started where the robot couldn't
place itself on the saved map is mapped in its own frame and stitched onto the map
afterwards (`align_session`).

Before a later session is merged in, its scans are compared with the map so far
(`detect_changes`): furniture that appeared, disappeared or moved, and newly mapped area.
Each change is accepted or rejected by the user. The map is then rebuilt (`compose`) from
the scans of every applied session, which sharpens walls seen again. In an accepted change
the newer session wins; in a rejected one the older map wins.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config import Config
from ..geometry import Pose
from ..nav.costmap import dilate
from ..nav.frontier import clusters
from .grid import OCC_THRESH, OccupancyGrid
from .matcher import LikelihoodField, _blur, match
from .refine import Keyframes, build_grid

STRONG_FREE = -1.2  # seen empty by at least 3 rays
MIN_CHANGE_CELLS = 8
MIN_CHANGE_EXTENT_M = 0.2
MIN_NEW_AREA_M2 = 0.5
JOIN_M = 0.15  # pieces this close belong to one object
MOVE_RADIUS_M = 6.0


# -- Stitching -----------------------------------------------------------------------


def transform_keyframes(kfs: Keyframes, t: Pose) -> Keyframes:
    return [(t.compose(p), pts) for p, pts in kfs]


def _session_points(cfg: Config, kfs: Keyframes, limit: int = 1500) -> np.ndarray:
    g = build_grid(cfg, kfs)
    pts = g.hit_points(g.occupied())
    if len(pts) > limit:
        pts = pts[np.random.default_rng(0).choice(len(pts), limit, replace=False)]
    return pts


def _overlap(base: OccupancyGrid, field: LikelihoodField, pts_w: np.ndarray) -> tuple[float, int]:
    """Fraction of points on the base map's known area that land on a base wall."""
    c = base.world_to_cell(pts_w)
    ok = base.in_bounds(c)
    c, w = c[ok], pts_w[ok]
    known = np.abs(base.logodds[c[:, 1], c[:, 0]]) > 0.05
    if known.sum() == 0:
        return 0.0, 0
    f = field._cellf(w[known])
    v = field.lookup_bilinear(f[:, 0], f[:, 1])
    return float((v > 0.3).mean()), int(known.sum())


def align_session(
    cfg: Config, base: OccupancyGrid, kfs: Keyframes, step_deg: float = 2.0, min_inliers: int = 60
) -> tuple[Pose, float] | None:
    """Find the transform placing a separately-mapped session on the base map.

    Global search: for every heading, the session's wall points are cross-correlated with the
    base map's walls (FFT, 10 cm cells); the best few candidates are refined by scan matching.
    Returns (transform, overlap) or None when there isn't enough overlap or the answer is
    ambiguous (e.g. a lone rectangular room).
    """
    pts = _session_points(cfg, kfs)
    bb = base.known_bbox(pad=0)
    if len(pts) < 50 or bb is None:
        return None
    k = 2  # coarse cell = 2 base cells
    r = base.res * k
    x0, y0, x1, y1 = bb
    x1, y1 = x0 + (x1 - x0 + k - 1) // k * k, y0 + (y1 - y0 + k - 1) // k * k
    occ = np.zeros((y1 - y0, x1 - x0), dtype=bool)
    sub = base.occupied()[y0:y1, x0:x1]
    occ[: sub.shape[0], : sub.shape[1]] = sub
    occ = occ.reshape(occ.shape[0] // k, k, occ.shape[1] // k, k).any(axis=(1, 3))
    fld = np.clip(_blur(occ.astype(np.float32), 1.0), 0, 1)
    bx0, by0 = base.origin[0] + x0 * base.res, base.origin[1] + y0 * base.res
    hf, wf = fld.shape
    span = float(np.ptp(pts, axis=0).max()) * 1.5 + 2 * r
    hh = wh = int(math.ceil(span / r)) + 1
    ny, nx = hf + hh, wf + wh
    ffld = np.fft.rfft2(fld, (ny, nx))

    cands = []
    for th in np.radians(np.arange(0.0, 360.0, step_deg)):
        c, s = math.cos(th), math.sin(th)
        q = pts @ np.array([[c, -s], [s, c]]).T
        qmin = q.min(axis=0)
        h = np.floor((q - qmin) / r).astype(int)
        img = np.zeros((hh, wh), dtype=np.float32)
        np.add.at(img, (h[:, 1], h[:, 0]), 1.0)
        corr = np.fft.irfft2(ffld * np.conj(np.fft.rfft2(img, (ny, nx))), (ny, nx))
        iy, ix = np.unravel_index(int(np.argmax(corr)), corr.shape)
        sy, sx = (iy if iy < hf else iy - ny), (ix if ix < wf else ix - nx)
        t = (bx0 + sx * r - qmin[0], by0 + sy * r - qmin[1])
        cands.append((float(corr[iy, ix]) / len(pts), Pose(t[0], t[1], float(th))))

    field = LikelihoodField(base, base.cell_to_world((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2),
                            max(bb[2] - bb[0], bb[3] - bb[1]) * base.res / 2 + 1.0)
    cands.sort(key=lambda c: -c[0])
    results: list[tuple[float, Pose]] = []
    for _, guess in cands[:6]:
        m = match(base, pts, guess, lin_window_m=0.25, ang_window_rad=math.radians(3), lin_step_cells=1,
                  ang_step_rad=math.radians(0.5), field=field, prior_weight=0.0)
        m = match(base, pts, m.pose, lin_window_m=0.06, ang_window_rad=math.radians(1),
                  ang_step_rad=math.radians(0.25), field=field, prior_weight=0.0)
        ratio, n = _overlap(base, field, m.pose.transform_points(pts))
        if n >= min_inliers:
            results.append((ratio, m.pose))
    if not results:
        return None
    results.sort(key=lambda r: -r[0])
    best_ratio, best = results[0]
    if best_ratio < 0.5:
        return None
    for ratio, p in results[1:]:
        far = p.distance_to(best.x, best.y) > 0.5 or abs(math.remainder(p.theta - best.theta, 2 * math.pi)) > math.radians(10)
        if far and ratio > best_ratio - 0.05:
            return None  # two different placements fit about equally well
    return best, best_ratio


# -- Change detection -------------------------------------------------------------------


@dataclass
class Change:
    kind: str  # added | removed | moved | new_area
    cells: np.ndarray  # flat grid indices
    to_cells: np.ndarray | None = None  # moved: where it went (cells = where it was)


def _blobs(mask: np.ndarray, join: int, min_cells: int) -> list[tuple[np.ndarray, np.ndarray]]:
    """Connected pieces of `mask` (pieces within `join` cells merged); (ys, xs) per blob."""
    if not mask.any():
        return []
    out = []
    for comp in clusters(dilate(mask, join), 1):
        xs, ys = np.array(comp).T
        keep = mask[ys, xs]
        if keep.sum() >= min_cells:
            out.append((ys[keep], xs[keep]))
    return out


def detect_changes(before: OccupancyGrid, after: OccupancyGrid) -> list[Change]:
    """What a new session (`after`, its scans only) sees differently from the map (`before`)."""
    known = (np.abs(before.logodds) > 0.05) | (np.abs(after.logodds) > 0.05)
    if not known.any():
        return []
    ys, xs = np.nonzero(known)
    y0, y1, x0, x1 = max(ys.min() - 4, 0), ys.max() + 5, max(xs.min() - 4, 0), xs.max() + 5
    lb, la = before.logodds[y0:y1, x0:x1], after.logodds[y0:y1, x0:x1]
    occ_b, occ_a = lb > OCC_THRESH, la > OCC_THRESH
    # A cell or so of wall jitter between sessions is not a change. Nor is a wall face that
    # was hidden (only grazed by rays) and is now in view: something new stands in open floor,
    # so the cells all round it must have been clearly free.
    def open_floor(lo):
        return ~dilate(~(lo < STRONG_FREE), 1)

    removed = occ_b & open_floor(la) & ~dilate(occ_a, 1)
    added = occ_a & open_floor(lb) & ~dilate(occ_b, 1)
    new_area = (np.abs(la) > 0.05) & ~dilate(np.abs(lb) > 0.05, 2)

    res = before.res
    join = int(round(JOIN_M / res))
    n = before.n

    def flat(ys, xs):
        return ((ys + y0) * n + (xs + x0)).astype(np.int64)

    def big(ys, xs):
        return max(np.ptp(xs), np.ptp(ys)) * res >= MIN_CHANGE_EXTENT_M

    gone = [b for b in _blobs(removed, join, MIN_CHANGE_CELLS) if big(*b)]
    came = [b for b in _blobs(added, join, MIN_CHANGE_CELLS) if big(*b)]
    # Something new also takes up the space inside its outline (never seen, so not "open
    # floor" any more).
    for j, (ay, ax) in enumerate(came):
        y_lo, y_hi, x_lo, x_hi = ay.min(), ay.max() + 1, ax.min(), ax.max() + 1
        iy, ix = np.nonzero((np.abs(la[y_lo:y_hi, x_lo:x_hi]) < 0.05) & (lb[y_lo:y_hi, x_lo:x_hi] < 0))
        came[j] = (np.concatenate([ay, iy + y_lo]), np.concatenate([ax, ix + x_lo]))

    def extent(ys, xs):
        return sorted((np.ptp(xs) * res, np.ptp(ys) * res))

    def alike(a, b):
        return all(abs(p - q) <= max(0.3, 0.25 * max(p, q)) for p, q in zip(extent(*a), extent(*b)))

    # Pair something that vanished with something the same size that appeared nearby: it moved.
    pairs = []
    for i, (gy, gx) in enumerate(gone):
        for j, (ay, ax) in enumerate(came):
            d = math.hypot(ax.mean() - gx.mean(), ay.mean() - gy.mean()) * res
            if d <= MOVE_RADIUS_M and alike(gone[i], came[j]):
                pairs.append((d, i, j))
    moved: dict[int, int] = {}
    for _, i, j in sorted(pairs):
        if i not in moved and j not in moved.values():
            moved[i] = j

    # Floor that was hidden under something now gone belongs to that change, not "new area".
    owner = np.full(removed.shape, -1, dtype=np.int32)
    for i, (gy, gx) in enumerate(gone):
        m = np.zeros(removed.shape, dtype=bool)
        m[gy, gx] = True
        owner[dilate(m, join + 2) & (owner < 0)] = i
    extra: dict[int, list] = {}
    rest = []
    min_new = int(MIN_NEW_AREA_M2 / (res * res))
    for ys, xs in _blobs(new_area, 2, 1):
        o = owner[ys, xs]
        if (o >= 0).any():
            extra.setdefault(int(o[o >= 0][0]), []).append(flat(ys, xs))
        elif len(ys) >= min_new:
            rest.append(Change("new_area", flat(ys, xs)))

    changes: list[Change] = []
    for i, (gy, gx) in enumerate(gone):
        cells = np.concatenate([flat(gy, gx), *extra.get(i, [])])
        if i in moved:
            ay, ax = came[moved[i]]
            changes.append(Change("moved", cells, flat(ay, ax)))
        else:
            changes.append(Change("removed", cells))
    changes += [Change("added", flat(ay, ax)) for j, (ay, ax) in enumerate(came) if j not in moved.values()]
    changes += rest
    return changes


def change_record(n: int, change: Change, accept: bool = True) -> dict:
    """How a change is stored with its session (JSON)."""
    return {"id": n, "kind": change.kind, "accept": accept, "cells": change.cells.tolist(),
            "to_cells": None if change.to_cells is None else change.to_cells.tolist()}


def from_record(c: dict) -> Change:
    to = c.get("to_cells")
    return Change(c["kind"], np.asarray(c["cells"], dtype=np.int64), None if to is None else np.asarray(to, dtype=np.int64))


def describe(change: Change, grid: OccupancyGrid) -> dict:
    """JSON-able summary: centre, bounding box, size, and the cells (world centres) to draw."""

    def info(cells):
        iy, ix = np.divmod(cells, grid.n)
        wx, wy = grid.cell_to_world(ix, iy)
        return {
            "x": round(float(wx.mean()), 3),
            "y": round(float(wy.mean()), 3),
            "bbox": [round(float(v), 3) for v in (wx.min(), wy.min(), wx.max(), wy.max())],
            "points": np.round(np.stack([wx, wy], 1), 3).tolist() if len(cells) <= 4000 else [],
        }

    out = {"kind": change.kind, **info(change.cells)}
    x0, y0, x1, y1 = out["bbox"]
    out["dims_m"] = [round(x1 - x0 + grid.res, 2), round(y1 - y0 + grid.res, 2)]
    out["size_m2"] = round(len(change.cells) * grid.res * grid.res, 2)
    if change.kind == "new_area":
        out["points"] = []  # drawn from its outline box; the map preview shows the rest
    if change.to_cells is not None:
        to = info(change.to_cells)
        out["to"] = {"x": to["x"], "y": to["y"], "bbox": to["bbox"]}
        out["to_points"] = to["points"]
        out["distance_m"] = round(math.hypot(to["x"] - out["x"], to["y"] - out["y"]), 2)
    return out


# -- The combined map ----------------------------------------------------------------------


def _paste(dst: OccupancyGrid, src: OccupancyGrid, cells: np.ndarray) -> None:
    if len(cells) == 0:
        return
    dst.logodds.flat[cells] = src.logodds.flat[cells]
    dst.hit_cnt.flat[cells] = src.hit_cnt.flat[cells]
    hs, ss = dst.hit_sum.reshape(-1, 2), src.hit_sum.reshape(-1, 2)
    hs[cells] = ss[cells]


def compose(cfg: Config, kfs: Keyframes, ids: np.ndarray, sessions: list[dict]) -> OccupancyGrid:
    """Build the map from the given sessions' scans, in order, honouring change decisions.

    `sessions` are dicts with "id" and, for later sessions, "changes": [{"cells", "to_cells", "accept"}]
    (as `change_record` makes them).
    """
    order = [s["id"] for s in sessions]
    grid = build_grid(cfg, [k for k, i in zip(kfs, ids) if i in order])

    def cells(c):
        return list(c["cells"]) + list(c.get("to_cells") or [])

    for n, s in enumerate(sessions):
        acc = [cells(c) for c in s.get("changes", []) if c.get("accept", True)]
        rej = [cells(c) for c in s.get("changes", []) if not c.get("accept", True)]
        if acc:
            mine = build_grid(cfg, [k for k, i in zip(kfs, ids) if i == s["id"]])
            _paste(grid, mine, np.concatenate(acc).astype(np.int64))
        if rej:
            earlier = set(order[:n])
            older = build_grid(cfg, [k for k, i in zip(kfs, ids) if i in earlier])
            _paste(grid, older, np.concatenate(rej).astype(np.int64))
    return grid
