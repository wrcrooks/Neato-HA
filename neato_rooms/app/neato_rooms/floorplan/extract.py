"""Occupancy grid -> architectural floorplan (walls with thickness, doors, room dimensions).

Pipeline (all geometry in metres):

1. Wall surface points: the mean LIDAR endpoint of every occupied cell (sub-cell accurate).
2. Local line orientation of each point (PCA over its neighbours) -> dominant wall direction.
   The plan is drawn in an *aligned* frame: the map rotated so the dominant walls are
   horizontal / vertical.
3. For each wall direction (0, 90 deg and any other well-supported angle): cluster points into
   lines, split lines into runs ("faces" = one visible side of a wall).
4. Faces are bridged across occluded gaps (things hidden behind furniture), split at gaps the
   robot could see through (doors / openings), and extended through occluded space to the
   wall they meet.
5. Classification: a face with the same-facing wall close behind it and unseen space between
   is furniture (sofa, bed). Opposite-facing faces close together are the two sides of one
   wall (-> thickness). Faces with "outside" behind them are exterior walls.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import numpy as np

from ..geometry import point_in_polygon
from ..nav.costmap import disk_offsets
from ..slam.grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid

INTERIOR_DEFAULT_M = 0.12
EXTERIOR_DEFAULT_M = 0.20
MAX_PAIR_GAP_M = 0.40  # two faces closer than this, back to back, are one wall
MIN_FACE_M = 0.25
DOOR_MIN_M, DOOR_MAX_M = 0.55, 1.30


@dataclass
class Face:
    id: int
    angle: float  # direction of the line in the aligned frame, [0, pi)
    v: float  # signed offset of the line (perpendicular coordinate in the line's frame)
    u0: float  # extent along the line
    u1: float
    side: int  # +1 / -1: which normal side the robot saw it from (free space); 0 = both
    n: int = 0
    kind: str = "wall"  # wall | exterior | fixture | furniture
    thickness: float = INTERIOR_DEFAULT_M
    visible: bool = True
    ext0: float = 0.0  # extra length added at each end to close corners
    ext1: float = 0.0

    # Frame helpers: line frame (u along, v across) <-> aligned frame.
    def to_xy(self, u: float, v: float) -> tuple[float, float]:
        c, s = math.cos(self.angle), math.sin(self.angle)
        return (u * c - v * s, u * s + v * c)

    def to_uv(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.angle), math.sin(self.angle)
        return (x * c + y * s, -x * s + y * c)

    @property
    def a(self):
        return self.to_xy(self.u0, self.v)

    @property
    def b(self):
        return self.to_xy(self.u1, self.v)

    @property
    def length(self) -> float:
        return self.u1 - self.u0

    def body(self) -> list[tuple[float, float]]:
        """Wall polygon: the face line extruded away from the free side by its thickness."""
        back = -self.side if self.side else 1
        t = self.thickness
        v0, v1 = (self.v, self.v + back * t) if self.side else (self.v - t / 2, self.v + t / 2)
        u0, u1 = self.u0 - self.ext0, self.u1 + self.ext1
        return [self.to_xy(u0, v0), self.to_xy(u1, v0), self.to_xy(u1, v1), self.to_xy(u0, v1)]


@dataclass
class Door:
    x: float
    y: float
    width: float
    angle: float  # wall direction
    side: int  # swing towards this normal side
    thickness: float


@dataclass
class RoomLabel:
    id: str
    name: str
    x: float
    y: float
    width_m: float | None
    depth_m: float | None
    polygon: list[list[float]]


@dataclass
class FloorPlan:
    rotation: float  # map -> aligned: aligned = R(-rotation) @ map
    faces: list[Face]
    doors: list[Door]
    rooms: list[RoomLabel]
    interior_area_m2: float
    bounds: tuple[float, float, float, float] = (0, 0, 0, 0)
    stats: dict = field(default_factory=dict)

    def walls(self) -> list[Face]:
        return [f for f in self.faces if f.visible and f.kind in ("wall", "exterior")]

    def as_dict(self) -> dict:
        out = {
            "rotation": self.rotation,
            "interior_area_m2": round(self.interior_area_m2, 2),
            "bounds": [round(b, 3) for b in self.bounds],
            "faces": [],
            "doors": [asdict(d) for d in self.doors],
            "rooms": [asdict(r) for r in self.rooms],
            "stats": self.stats,
        }
        for f in self.faces:
            out["faces"].append({
                "id": f.id, "kind": f.kind, "visible": f.visible, "thickness": round(f.thickness, 3),
                "a": [round(c, 3) for c in f.a], "b": [round(c, 3) for c in f.b],
                "angle": round(f.angle, 4), "length": round(f.length, 3),
                "body": [[round(x, 3), round(y, 3)] for x, y in f.body()],
            })
        return out


# ---------------------------------------------------------------------------------------
# Helpers


def _rot(pts: np.ndarray, ang: float) -> np.ndarray:
    c, s = math.cos(ang), math.sin(ang)
    return pts @ np.array([[c, -s], [s, c]]).T


class _GridView:
    """Classifies aligned-frame points using the occupancy grid."""

    def __init__(self, grid: OccupancyGrid, rotation: float):
        self.grid = grid
        self.rotation = rotation
        self.cls = grid.classes()
        self.outside = _outside_mask(self.cls)

    def _cells(self, pts_aligned: np.ndarray):
        m = _rot(np.asarray(pts_aligned, dtype=float).reshape(-1, 2), self.rotation)
        c = self.grid.world_to_cell(m)
        ok = self.grid.in_bounds(c)
        return c, ok

    def classes_at(self, pts: np.ndarray) -> np.ndarray:
        c, ok = self._cells(pts)
        out = np.full(len(c), UNKNOWN, dtype=np.uint8)
        out[ok] = self.cls[c[ok, 1], c[ok, 0]]
        return out

    def free_fraction(self, pts: np.ndarray) -> float:
        if len(pts) == 0:
            return 0.0
        return float((self.classes_at(pts) == FREE).mean())

    def outside_fraction(self, pts: np.ndarray) -> float:
        c, ok = self._cells(pts)
        if len(c) == 0:
            return 0.0
        out = np.ones(len(c), dtype=bool)
        out[ok] = self.outside[c[ok, 1], c[ok, 0]]
        return float(out.mean())


def _outside_mask(cls: np.ndarray) -> np.ndarray:
    """Unknown cells connected to the edge of the known area (i.e. outside the house)."""
    known = cls != UNKNOWN
    ys, xs = np.nonzero(known)
    out = np.ones(cls.shape, dtype=bool)
    if len(xs) == 0:
        return out
    pad = 16
    y0, y1 = max(ys.min() - pad, 0), min(ys.max() + pad + 1, cls.shape[0])
    x0, x1 = max(xs.min() - pad, 0), min(xs.max() + pad + 1, cls.shape[1])
    unk = cls[y0:y1, x0:x1] == UNKNOWN
    cur = np.zeros_like(unk)
    cur[0, :], cur[-1, :], cur[:, 0], cur[:, -1] = unk[0, :], unk[-1, :], unk[:, 0], unk[:, -1]
    while True:
        grow = cur.copy()
        grow[1:, :] |= cur[:-1, :]
        grow[:-1, :] |= cur[1:, :]
        grow[:, 1:] |= cur[:, :-1]
        grow[:, :-1] |= cur[:, 1:]
        grow &= unk
        if (grow == cur).all():
            break
        cur = grow
    out[y0:y1, x0:x1] = cur
    return out


def _surface_points(grid: OccupancyGrid, r: int = 2):
    """Wall surfaces as the robot saw them: occupied cells touching free space.

    Returns (points (N,2) sub-cell accurate, line angle in [0, pi), normal (N,2) unit vector
    pointing into the free side, strength in [0, 1]).
    """
    cls = grid.classes()
    occ, free = cls == OCCUPIED, cls == FREE
    bbox = grid.known_bbox(pad=r + 2)
    if bbox is None:
        z = np.zeros((0, 2))
        return z, np.zeros(0), z, np.zeros(0)
    x0, y0, x1, y1 = bbox
    o, fr = occ[y0:y1, x0:x1], free[y0:y1, x0:x1].astype(float)
    nb = np.zeros_like(o)
    nb[1:, :] |= fr[:-1, :] > 0
    nb[:-1, :] |= fr[1:, :] > 0
    nb[:, 1:] |= fr[:, :-1] > 0
    nb[:, :-1] |= fr[:, 1:] > 0
    surf = o & nb
    h, w = o.shape
    fp = np.pad(fr, r)
    nx = np.zeros((h, w))
    ny = np.zeros((h, w))
    norm = 0.0
    for dx, dy in disk_offsets(r):
        if dx == 0 and dy == 0:
            continue
        v = fp[r + dy : r + dy + h, r + dx : r + dx + w]
        nx += v * dx
        ny += v * dy
        norm += math.hypot(dx, dy) / 2  # a free half-plane gives roughly this magnitude
    full = np.zeros(occ.shape, dtype=bool)
    full[y0:y1, x0:x1] = surf
    pts = grid.hit_points(full)
    iy, ix = np.nonzero(surf)
    n = np.stack([nx[iy, ix], ny[iy, ix]], axis=1)
    mag = np.linalg.norm(n, axis=1)
    unit = n / np.maximum(mag, 1e-9)[:, None]
    # Direction from a PCA line fit over neighbouring surface points (precise); the normal
    # only decides which side the face looks into. Points only "see" neighbours facing the
    # same way, otherwise the two faces of a thin wall (2-3 cells apart) blur together.
    # Two binnings (offset 45 deg) so diagonal walls aren't split across bins.
    nang = np.arctan2(unit[:, 1], unit[:, 0])
    theta = np.zeros(len(pts))
    lin = np.full(len(pts), -1.0)
    for offset in (0.0, math.pi / 4):
        b = np.floor(((nang + offset) % (2 * math.pi)) / (math.pi / 2)).astype(int)
        for k in range(4):
            sel = b == k
            if sel.sum() < 3:
                continue
            m = np.zeros_like(surf)
            m[iy[sel], ix[sel]] = True
            th_k, lin_k = _pca_orientation(m, iy[sel], ix[sel], pts[sel], r=3)
            idx = np.nonzero(sel)[0]
            better = lin_k > lin[idx]
            theta[idx[better]] = th_k[better]
            lin[idx[better]] = lin_k[better]
    strength = np.clip(lin, 0, 1) * np.clip(mag / norm, 0, 1) ** 0.5
    return pts, theta, unit, strength


def _pca_orientation(mask, iy, ix, pts, r=3):
    h, w = mask.shape
    px = np.zeros((h, w))
    py = np.zeros((h, w))
    px[iy, ix] = pts[:, 0] - pts[:, 0].mean()
    py[iy, ix] = pts[:, 1] - pts[:, 1].mean()
    mp, xp, yp = np.pad(mask, r), np.pad(px, r), np.pad(py, r)
    acc = {k: np.zeros((h, w)) for k in ("n", "x", "y", "xx", "xy", "yy")}
    for dx, dy in disk_offsets(r):
        sl = (slice(r + dy, r + dy + h), slice(r + dx, r + dx + w))
        mm, xx, yy = mp[sl], xp[sl], yp[sl]
        acc["n"] += mm
        acc["x"] += xx
        acc["y"] += yy
        acc["xx"] += xx * xx
        acc["xy"] += xx * yy
        acc["yy"] += yy * yy
    n = np.maximum(acc["n"], 1)
    mx, my = acc["x"] / n, acc["y"] / n
    cxx = acc["xx"] / n - mx * mx
    cxy = acc["xy"] / n - mx * my
    cyy = acc["yy"] / n - my * my
    theta = np.mod(0.5 * np.arctan2(2 * cxy, cxx - cyy), math.pi)
    disc = np.sqrt(((cxx - cyy) / 2) ** 2 + cxy**2)
    tr = cxx + cyy
    lin = np.where((tr > 1e-12) & (acc["n"] >= 3), 2 * disc / np.maximum(tr, 1e-12), 0.0)
    return theta[iy, ix], np.clip(lin[iy, ix], 0, 1)


def sharpen_angle(pts: np.ndarray, guess: float, span_deg: float = 3.0, step_deg: float = 0.02) -> float:
    """Refine the wall direction: the angle at which wall points project into the sharpest
    x and y histograms (walls exactly axis-aligned). Far more precise than averaging local
    orientations, which are biased by jagged edges."""
    if len(pts) < 20:
        return guess
    best, best_score = guess, -1.0
    for d in np.arange(-span_deg, span_deg + 1e-9, step_deg):
        a = guess + math.radians(d)
        q = _rot(pts, -a)
        score = 0.0
        for k in (0, 1):
            v = q[:, k]
            hist = np.bincount(((v - v.min()) / 0.01).astype(int))
            score += float((hist.astype(float) ** 2).sum())
        if score > best_score:
            best, best_score = a, score
    return float((best + math.pi / 4) % (math.pi / 2) - math.pi / 4)


def dominant_angle(theta: np.ndarray, lin: np.ndarray) -> float:
    """Manhattan direction (mod 90 deg) of the wall points, in (-pi/4, pi/4]."""
    good = lin > 0.6
    if good.sum() < 5:
        return 0.0
    t, w = theta[good], lin[good] ** 2
    ang = np.angle(np.sum(w * np.exp(4j * t))) / 4
    # Refine on points that belong to this system only.
    d = np.abs((t - ang + math.pi / 4) % (math.pi / 2) - math.pi / 4)
    near = d < math.radians(8)
    if near.sum() >= 5:
        ang = np.angle(np.sum(w[near] * np.exp(4j * t[near]))) / 4
    return float(ang)


# ---------------------------------------------------------------------------------------
# Face extraction


def _faces_for_angle(phi, pts, theta, normal, strength, res, start_id) -> list[Face]:
    """Faces along direction `phi` (aligned frame), one set per facing side."""
    d = np.abs((theta - phi + math.pi / 2) % math.pi - math.pi / 2)
    perp = np.array([-math.sin(phi), math.cos(phi)])
    sel = (d < math.radians(15)) & (strength > 0.3)
    faces: list[Face] = []
    for side in (1, -1):
        m = sel & (np.sign(normal @ perp) == side)
        if m.sum() < 4:
            continue
        uv = _rot(pts[m], -phi)
        v_all = uv[:, 1]
        lo, hi = v_all.min() - 0.05, v_all.max() + 0.05
        nb = max(int((hi - lo) / 0.01), 1)
        hist, edges = np.histogram(v_all, bins=nb, range=(lo, hi))
        k = np.exp(-(np.arange(-3, 4) ** 2) / (2 * 1.5**2))
        sm = np.convolve(hist, k, mode="same")
        peaks = [i for i in range(nb) if sm[i] >= 2.5 and (i == 0 or sm[i] >= sm[i - 1]) and (i == nb - 1 or sm[i] > sm[i + 1])]
        peaks.sort(key=lambda i: -sm[i])
        centers: list[float] = []
        for i in peaks:
            c = (edges[i] + edges[i + 1]) / 2
            if all(abs(c - o) > 0.05 for o in centers):
                centers.append(c)
        taken = np.zeros(len(uv), dtype=bool)
        for c in centers:
            near = (np.abs(uv[:, 1] - c) < 0.035) & ~taken
            taken |= near
            grp = uv[near]
            if len(grp) < 4:
                continue
            g = grp[np.argsort(grp[:, 0])]
            splits = np.nonzero(np.diff(g[:, 0]) > max(3 * res, 0.15))[0]
            for run in np.split(g, splits + 1):
                if len(run) < 4 or run[-1, 0] - run[0, 0] < 0.15:
                    continue
                # Surface cells are ~one cell apart, so the run covers half a cell past its ends.
                u0, u1 = run[0, 0] - res / 2, run[-1, 0] + res / 2
                faces.append(Face(start_id + len(faces), phi, float(np.median(run[:, 1])), float(u0), float(u1), side, len(run)))
    return faces


def _samples(f: Face, dv: float, u0=None, u1=None, step=0.04) -> np.ndarray:
    u0 = f.u0 if u0 is None else u0
    u1 = f.u1 if u1 is None else u1
    if u1 <= u0:
        return np.zeros((0, 2))
    us = np.arange(u0 + step / 2, u1, step)
    return np.array([f.to_xy(u, f.v + dv) for u in us])


def _same_line(a: Face, b: Face, tol=0.07) -> bool:
    return abs(a.angle - b.angle) < 1e-6 and a.side == b.side and abs(a.v - b.v) < tol


def _merge_and_split(faces: list[Face], gv: _GridView) -> tuple[list[Face], list[Door]]:
    """Bridge occluded gaps between collinear faces; record seen-through gaps as openings."""
    out: list[Face] = []
    doors: list[Door] = []
    used = [False] * len(faces)
    for i, f in sorted(enumerate(faces), key=lambda t: (t[1].angle, t[1].side, t[1].v, t[1].u0)):
        if used[i]:
            continue
        group = [f]
        used[i] = True
        # Grow the group against its running (point-weighted) line, so a face a few cm off
        # the first one still joins once the group's estimate settles between them.
        grew = True
        while grew:
            grew = False
            ref = Face(f.id, f.angle, sum(g.v * g.n for g in group) / max(sum(g.n for g in group), 1), 0, 0, f.side)
            for j, g in enumerate(faces):
                if not used[j] and _same_line(ref, g):
                    group.append(g)
                    used[j] = grew = True
        group.sort(key=lambda x: x.u0)
        start = len(out)
        cur = group[0]
        for nxt in group[1:]:
            gap0, gap1 = cur.u1, nxt.u0
            if gap1 <= gap0 + 0.02:
                cur = _join(cur, nxt)
                continue
            # Sample just inside the wall line: free there means the robot saw through the gap.
            line = _samples(cur, -(cur.side or 1) * 0.03, gap0, gap1, step=0.02)
            free = gv.classes_at(line) == FREE if len(line) else np.zeros(0, dtype=bool)
            run, best, best_end = 0, 0, 0
            for i, fr in enumerate(free):
                run = run + 1 if fr else 0
                if run > best:
                    best, best_end = run, i
            if best * 0.02 >= 0.3:
                width = best * 0.02
                mid = gap0 + (best_end + 1 - best / 2) * 0.02
                if width >= 0.75 * (gap1 - gap0):
                    # Clear opening: the jambs are the face ends (sub-cell accurate).
                    width, mid = gap1 - gap0, (gap0 + gap1) / 2
                if DOOR_MIN_M <= width <= DOOR_MAX_M:
                    x, y = cur.to_xy(mid, cur.v)
                    doors.append(Door(x, y, width, cur.angle, cur.side or 1, INTERIOR_DEFAULT_M))
                out.append(cur)
                cur = nxt
            else:
                cur = _join(cur, nxt)
        out.append(cur)
        # Pieces of one wall split by doorways share its line.
        v = sum(g.v * g.n for g in out[start:]) / max(sum(g.n for g in out[start:]), 1)
        for g in out[start:]:
            g.v = v
    return out, doors


def _join(a: Face, b: Face) -> Face:
    n = a.n + b.n
    v = (a.v * a.n + b.v * b.n) / max(n, 1)
    return Face(a.id, a.angle, v, min(a.u0, b.u0), max(a.u1, b.u1), a.side, n)


def _intersect(f: Face, g: Face):
    """Intersection of the two face lines as (u along f, u along g), or None if parallel."""
    da = (math.cos(f.angle), math.sin(f.angle))
    db = (math.cos(g.angle), math.sin(g.angle))
    den = da[0] * db[1] - da[1] * db[0]
    if abs(den) < math.sin(math.radians(20)):
        return None
    pa, pb = f.to_xy(0, f.v), g.to_xy(0, g.v)
    dx, dy = pb[0] - pa[0], pb[1] - pa[1]
    t = (dx * db[1] - dy * db[0]) / den
    s = (dx * da[1] - dy * da[0]) / den
    return t, s


def _close_corners(faces: list[Face], gv: _GridView, max_ext=2.5) -> None:
    """Extend / trim face ends to the wall they run into, through unseen space only."""
    faces = [f for f in faces if f.kind != "furniture"]
    for f in faces:
        orig = (f.u0, f.u1)
        for end in (0, 1):
            u_end = orig[end]
            direction = -1 if end == 0 else 1
            best = None
            for g in faces:
                if g is f:
                    continue
                hit = _intersect(f, g)
                if hit is None:
                    continue
                t, s = hit
                if not (g.u0 - 0.15 <= s <= g.u1 + 0.15):
                    # The other wall may also be hidden near the corner (e.g. a bed in it).
                    g_end, g_dir = (g.u0, -1) if s < g.u0 else (g.u1, 1)
                    g_dist = (s - g_end) * g_dir
                    if g_dist > max_ext or gv.free_fraction(_samples(g, -(g.side or 1) * 0.03, *sorted((g_end, s)))) > 0.15:
                        continue
                dist = (t - u_end) * direction
                if -0.2 <= dist <= max_ext and (best is None or abs(dist) < abs(best[0])):
                    best = (dist, t)
            if best is None:
                continue
            dist, t = best
            if dist > 0.05:
                lo, hi = sorted((u_end, t))
                # Only through space the robot never saw (occluded / wall bodies).
                if gv.free_fraction(_samples(f, -(f.side or 1) * 0.03, lo + 0.05, hi - 0.05)) > 0.15:
                    continue
            if end == 0:
                f.u0 = t
            else:
                f.u1 = t


def _overlap(a: Face, b: Face) -> float:
    return max(0.0, min(a.u1, b.u1) - max(a.u0, b.u0))


def _unseen_span(g: Face, gv: _GridView, max_ext: float = 2.5) -> tuple[float, float]:
    """g's extent if continued along its line through space the robot never saw."""
    inside = g.v - (g.side or 1) * 0.03  # just behind the surface, so cell rounding can't
                                          # put the sample in the room

    def reach(u_end, direction):
        u, frees = u_end, 0
        while abs(u - u_end) < max_ext:
            nxt = u + direction * 0.05
            if gv.classes_at(np.array([g.to_xy(nxt, inside)]))[0] == FREE:
                frees += 1
                if frees >= 2:
                    break
            else:
                frees = 0
                u = nxt
            if frees:
                u = nxt
        return u - direction * 0.05 * frees
    return reach(g.u0, -1), reach(g.u1, 1)


def _has_wall_partner(f: Face, faces: list[Face]) -> bool:
    """The other side of the same wall is right behind this face."""
    for g in faces:
        if g is f or g.side != -f.side or abs(g.angle - f.angle) > 1e-6:
            continue
        d = (g.v - f.v) * -f.side
        if 0.03 <= d <= MAX_PAIR_GAP_M and _overlap(f, g) >= 0.3 * min(f.length, g.length):
            return True
    return False


def _classify_furniture(faces: list[Face], gv: _GridView) -> None:
    """A face with the same-facing wall behind it (seen, or hidden behind this very object)
    and unseen space in between is the front of furniture: sofa, bed, cabinet."""
    spans = {id(g): _unseen_span(g, gv) for g in faces}
    footprints = []  # (front face, depth to the wall behind it)
    for f in faces:
        if f.side == 0 or _has_wall_partner(f, faces):
            continue
        back = -f.side
        for g in faces:
            if g is f or g.side != f.side or abs(g.angle - f.angle) > 1e-6:
                continue
            d = (g.v - f.v) * back
            if not 0.15 <= d <= 3.0:
                continue
            u0, u1 = spans[id(g)]
            if min(f.u1, u1) - max(f.u0, u0) < 0.6 * f.length:
                continue
            strip = np.concatenate([_samples(f, back * dv) for dv in np.arange(0.1, d - 0.05, 0.1)] or [np.zeros((0, 2))])
            if gv.free_fraction(strip) < 0.5:
                f.kind, f.visible = "furniture", False
                footprints.append((f, d))
                break
    # Its ends are too: the short sides running back from the front to the wall.
    for f, d in footprints:
        lo_v, hi_v = sorted((f.v, f.v - f.side * d))
        for g in faces:
            if g.kind == "furniture" or abs(abs(math.remainder(g.angle - f.angle, math.pi)) - math.pi / 2) > 1e-3:
                continue
            ends = [f.to_uv(*g.a), f.to_uv(*g.b)]
            if all(f.u0 - 0.1 <= u <= f.u1 + 0.1 and lo_v - 0.1 <= v <= hi_v + 0.1 for u, v in ends):
                g.kind, g.visible = "furniture", False
    # The back of a piece of furniture is furniture too (e.g. a sofa seen from behind).
    for f in faces:
        if f.kind == "furniture" or f.side == 0 or _has_wall_partner(f, faces):
            continue
        for g in faces:
            if g.kind != "furniture" or g.side != -f.side or abs(g.angle - f.angle) > 1e-6:
                continue
            d = (g.v - f.v) * -f.side
            if 0.15 <= d <= 1.6 and _overlap(f, g) >= 0.5 * f.length:
                f.kind, f.visible = "furniture", False
                break


def _classify(faces: list[Face], gv: _GridView) -> None:
    by_angle: dict[float, list[Face]] = {}
    for f in faces:
        by_angle.setdefault(round(f.angle, 6), []).append(f)
    for group in by_angle.values():
        for f in group:
            if f.kind == "furniture":
                continue
            if f.side == 0:
                f.thickness = 0.10
                continue
            back = -f.side
            # Thickness from the opposite face of the same wall.
            partner = None
            for g in group:
                if g is f or g.side != -f.side or g.kind == "furniture":
                    continue
                d = (g.v - f.v) * back
                if 0.03 <= d and _overlap(f, g) >= 0.3 * min(f.length, g.length):
                    if partner is None or d < partner[0]:
                        partner = (d, g)
            if partner and partner[0] <= MAX_PAIR_GAP_M:
                f.thickness = partner[0]
                continue
            if partner and partner[0] <= 1.6 and f.length < 3.0:
                strip = _samples(f, back * partner[0] / 2)
                if gv.free_fraction(strip) < 0.2:
                    f.kind = "fixture"  # free-standing (kitchen island, counter)
                    f.thickness = partner[0]
                    continue
            behind = np.concatenate([_samples(f, back * dv) for dv in (0.3, 0.5, 0.8)])
            if gv.outside_fraction(behind) > 0.6:
                f.kind = "exterior"
                f.thickness = EXTERIOR_DEFAULT_M
            else:
                f.thickness = INTERIOR_DEFAULT_M
    # A "free-standing" object the robot can't get behind (a sofa a hand's width from the
    # wall) is furniture, not an island: hide it with all its faces.
    walls = [g for g in faces if g.kind in ("wall", "exterior")]
    for f in faces:
        if f.kind != "fixture":
            continue
        if not any(abs(h.angle - f.angle) < 1e-6 and h.side == -f.side and 0 < (h.v - f.v) * f.side <= 0.5
                   and _overlap(f, h) >= min(0.3, 0.5 * f.length) for h in walls):
            continue
        pts = np.array([f.a, f.b, f.to_xy(f.u0, f.v - f.side * f.thickness), f.to_xy(f.u1, f.v - f.side * f.thickness)])
        lo, hi = pts.min(axis=0) - 0.1, pts.max(axis=0) + 0.1
        for g in faces:
            mid = (np.array(g.a) + np.array(g.b)) / 2
            if g.kind == "fixture" and (lo <= mid).all() and (mid <= hi).all():
                g.kind, g.visible = "furniture", False


def _corner_fill(faces: list[Face]) -> None:
    """Lengthen each wall body past its corners by the thickness of the wall it meets."""
    walls = [f for f in faces if f.visible and f.kind in ("wall", "exterior")]
    for f in walls:
        for end in (0, 1):
            u_end = f.u0 if end == 0 else f.u1
            best = 0.0
            for g in walls:
                if g is f:
                    continue
                hit = _intersect(f, g)
                if hit is None:
                    continue
                t, s = hit
                if abs(t - u_end) < 0.06 and g.u0 - 0.06 <= s <= g.u1 + 0.06:
                    best = max(best, g.thickness)
            if end == 0:
                f.ext0 = best
            else:
                f.ext1 = best


def _is_corridor(d: Door, faces: list[Face]) -> bool:
    """Opening whose jambs continue as walls on one side = corridor end, not a door."""
    c, s = math.cos(d.angle), math.sin(d.angle)
    jambs = [(d.x - c * d.width / 2, d.y - s * d.width / 2), (d.x + c * d.width / 2, d.y + s * d.width / 2)]
    for sgn in (1, -1):
        ok = 0
        for jx, jy in jambs:
            for f in faces:
                if f.kind == "furniture" or _intersect(f, Face(0, d.angle, 0, 0, 0, 1)) is None:
                    continue
                fc, fs = math.cos(f.angle), math.sin(f.angle)
                u = jx * fc + jy * fs
                v = -jx * fs + jy * fc
                if abs(v - f.v) > d.thickness + 0.1:
                    continue
                # Wall running away from the jamb on side `sgn` for at least 0.5 m.
                nx, ny = -s * sgn, c * sgn
                along = nx * fc + ny * fs
                reach = (f.u1 - u) if along > 0 else (u - f.u0)
                if reach >= 0.5 and (f.u0 - 0.15 <= u <= f.u1 + 0.15):
                    ok += 1
                    break
        if ok == 2:
            return True
    return False


def _between_walls(d: Door, faces: list[Face]) -> bool:
    c, s = math.cos(d.angle), math.sin(d.angle)
    jambs = [(d.x - c * d.width / 2, d.y - s * d.width / 2), (d.x + c * d.width / 2, d.y + s * d.width / 2)]
    for jx, jy in jambs:
        found = False
        for f in faces:
            if f.kind not in ("wall", "exterior") or abs(f.angle - d.angle) > 1e-6:
                continue
            u, v = jx * c + jy * s, -jx * s + jy * c
            if abs(v - f.v) < 0.1 and min(abs(u - f.u0), abs(u - f.u1)) < 0.15:
                found = True
                break
        if not found:
            return False
    return True


def _refine_door_width(d: Door, pts: np.ndarray) -> None:
    """Jamb positions from the LIDAR hits on the wall ends either side of the opening."""
    c, s = math.cos(d.angle), math.sin(d.angle)
    u = pts[:, 0] * c + pts[:, 1] * s
    v = -pts[:, 0] * s + pts[:, 1] * c
    uc, vc = d.x * c + d.y * s, -d.x * s + d.y * c
    vmid = vc - d.side * d.thickness / 2
    band = np.abs(v - vmid) <= d.thickness / 2 + 0.02
    half = d.width / 2
    left = band & (u < uc) & (u > uc - half - 0.15)
    right = band & (u > uc) & (u < uc + half + 0.15)
    if left.any() and right.any():
        ul, ur = u[left].max(), u[right].min()
        if DOOR_MIN_M - 0.1 <= ur - ul <= DOOR_MAX_M + 0.1:
            d.width = float(ur - ul)
            m = (ul + ur) / 2
            d.x, d.y = m * c - vc * s, m * s + vc * c


def _dedupe_doors(doors: list[Door], faces: list[Face]) -> list[Door]:
    out: list[Door] = []
    for d in doors:
        if any(math.hypot(d.x - o.x, d.y - o.y) < 0.3 for o in out):
            continue
        out.append(d)
    out = [d for d in out if _between_walls(d, faces) and not _is_corridor(d, faces)]
    walls = [f for f in faces if f.kind in ("wall", "exterior")]
    for d in out:
        # Use the thickness of the wall the door sits in.
        for f in walls:
            if abs(f.angle - d.angle) < 1e-6:
                c, s = math.cos(f.angle), math.sin(f.angle)
                v = -d.x * s + d.y * c
                if abs(v - f.v) < 0.05 or abs(v - (f.v - (f.side or 1) * f.thickness)) < 0.05:
                    d.thickness = f.thickness
                    break
    return out


# ---------------------------------------------------------------------------------------
# Rooms


def _ray_to_walls(x, y, dx, dy, walls: list[Face], max_d=15.0) -> float | None:
    best = None
    for f in walls:
        ax, ay = f.a
        bx, by = f.b
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / den
        u = ((ax - x) * dy - (ay - y) * dx) / den
        if 0 < t < max_d and -0.02 <= u <= 1.02 and (best is None or t < best):
            best = t
    return best


def _ray_exit(x, y, dx, dy, poly: np.ndarray) -> float | None:
    """Distance along the ray to where it leaves `poly` (starting inside)."""
    best = None
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / den
        u = ((ax - x) * dy - (ay - y) * dx) / den
        if t > 0 and 0 <= u <= 1 and (best is None or t < best):
            best = t
    return best


def _span(x, y, dx, dy, walls, poly) -> float | None:
    """Distance to the room boundary: the wall if there is one near the drawn room edge,
    otherwise the drawn edge itself (open-plan spaces, archways)."""
    w = _ray_to_walls(x, y, dx, dy, walls)
    e = _ray_exit(x, y, dx, dy, poly)
    if e is None:
        return w
    if w is not None and w <= e + 0.5:
        return w
    # Through a doorway / open-plan edge: stop on the line of the wall the opening is in.
    best = None
    for f in walls:
        fc, fs = math.cos(f.angle), math.sin(f.angle)
        den = dx * -fs + dy * fc  # ray direction . face normal
        if abs(den) < 0.9:
            continue
        t = (f.v - (-x * fs + y * fc)) / den
        if 0 < t and abs(t - e) <= 0.5 and (best is None or t < best):
            best = t
    return best if best is not None else e


def _label_point(mask: np.ndarray) -> tuple[int, int] | None:
    """Most open cell of a mask (repeated erosion; like a pole of inaccessibility)."""
    if not mask.any():
        return None
    cur = mask
    while True:
        er = cur.copy()
        er[1:, :] &= cur[:-1, :]
        er[:-1, :] &= cur[1:, :]
        er[:, 1:] &= cur[:, :-1]
        er[:, :-1] &= cur[:, 1:]
        er[0, :] = er[-1, :] = False
        er[:, 0] = er[:, -1] = False
        if not er.any():
            break
        cur = er
    ys, xs = np.nonzero(cur)
    i = int(np.argmin((xs - xs.mean()) ** 2 + (ys - ys.mean()) ** 2))
    return int(xs[i]), int(ys[i])


def _room_labels(areas, gv: _GridView, walls: list[Face], rotation: float) -> list[RoomLabel]:
    grid = gv.grid
    bbox = grid.known_bbox(pad=2)
    if bbox is None:
        return []
    x0, y0, x1, y1 = bbox
    free = gv.cls[y0:y1, x0:x1] == FREE
    ix, iy = np.meshgrid(np.arange(x0, x1), np.arange(y0, y1))
    fx, fy = grid.cell_to_world(ix, iy)
    out = []
    for a in areas:
        if a.get("kind") != "room":
            continue
        poly = np.array(a["polygon"], dtype=float)
        pa = _rot(poly, -rotation)
        inside = point_in_polygon(fx, fy, a["polygon"]) & free
        if inside.sum() < 5:
            cx, cy = pa.mean(axis=0)
            out.append(RoomLabel(a["id"], a["name"], float(cx), float(cy), None, None, pa.round(3).tolist()))
            continue
        lp = _label_point(inside)
        lx, ly = _rot(np.array([[fx[lp[1], lp[0]], fy[lp[1], lp[0]]]]), -rotation)[0]
        pts = _rot(np.stack([fx[inside], fy[inside]], 1), -rotation)
        if len(pts) > 150:
            pts = pts[np.random.default_rng(0).choice(len(pts), 150, replace=False)]
        widths, depths = [], []
        for x, y in pts:
            l, r = _span(x, y, -1, 0, walls, pa), _span(x, y, 1, 0, walls, pa)
            d, u = _span(x, y, 0, -1, walls, pa), _span(x, y, 0, 1, walls, pa)
            if l and r:
                widths.append(l + r)
            if d and u:
                depths.append(d + u)
        # Room size = the dominant (most common) span: robust to alcoves and doorways.
        out.append(RoomLabel(a["id"], a["name"], float(lx), float(ly), _mode(widths), _mode(depths), pa.round(3).tolist()))
    return out


def _interior_area(gv: _GridView, faces: list[Face]) -> float:
    """Floor area inside the exterior walls (wall bodies inferred behind furniture count as
    barriers, so space hidden under a bed isn't mistaken for outside)."""
    grid = gv.grid
    # The crop must enclose the wall bodies, not just the scanned area: an unseen corner
    # (behind a bed) of a house drawn at an angle can stick out of the scanned bbox.
    pad = int(math.ceil(0.4 / grid.res))
    bbox = grid.known_bbox(pad=pad)
    if bbox is None:
        return 0.0
    x0, y0, x1, y1 = bbox
    bodies = [_rot(np.array(f.body()), gv.rotation) for f in faces if f.visible and f.kind in ("wall", "exterior")]
    if bodies:
        lo = grid.world_to_cell(np.min([b.min(axis=0) for b in bodies], axis=0)[None])[0] - pad
        hi = grid.world_to_cell(np.max([b.max(axis=0) for b in bodies], axis=0)[None])[0] + pad + 1
        x0, y0 = max(min(x0, int(lo[0])), 0), max(min(y0, int(lo[1])), 0)
        x1, y1 = min(max(x1, int(hi[0])), grid.n), min(max(y1, int(hi[1])), grid.n)
    cls = gv.cls[y0:y1, x0:x1]
    ix, iy = np.meshgrid(np.arange(x0, x1), np.arange(y0, y1))
    wx, wy = grid.cell_to_world(ix, iy)
    barrier = cls == OCCUPIED
    exterior = np.zeros_like(barrier)
    for f in faces:
        if not f.visible or f.kind not in ("wall", "exterior"):
            continue
        body = _rot(np.array(f.body()), gv.rotation)
        bx0, by0 = body.min(axis=0) - grid.res
        bx1, by1 = body.max(axis=0) + grid.res
        sel = (wx >= bx0) & (wx <= bx1) & (wy >= by0) & (wy <= by1)
        hit = np.zeros_like(barrier)
        hit[sel] = point_in_polygon(wx[sel], wy[sel], body.tolist())
        barrier |= hit
        if f.kind == "exterior":
            exterior |= hit
    passable = (cls == UNKNOWN) & ~barrier
    out = np.zeros_like(passable)
    out[0, :], out[-1, :], out[:, 0], out[:, -1] = passable[0, :], passable[-1, :], passable[:, 0], passable[:, -1]
    while True:
        grow = out.copy()
        grow[1:, :] |= out[:-1, :]
        grow[:-1, :] |= out[1:, :]
        grow[:, 1:] |= out[:, :-1]
        grow[:, :-1] |= out[:, 1:]
        grow &= passable
        if (grow == out).all():
            break
        out = grow
    return float((~out & ~exterior).sum()) * grid.res**2


def _mode(vals: list[float]) -> float | None:
    if len(vals) < 3:
        return None
    v = np.array(vals)
    hist, edges = np.histogram(v, bins=np.arange(v.min(), v.max() + 0.06, 0.05))
    i = int(np.argmax(hist))
    sel = v[(v >= edges[i] - 0.03) & (v <= edges[i + 1] + 0.03)]
    return float(np.median(sel))


# ---------------------------------------------------------------------------------------


def same_override(a: dict, b: dict) -> bool:
    """Both records refer to the same face: same place and facing the same way (so the two
    sides of a thin wall are distinct)."""
    da = abs((a["normal"] - b["normal"] + math.pi) % (2 * math.pi) - math.pi)
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < 0.25 and da < math.radians(10)


def _match_override(f: Face, overrides: list[dict], rotation: float) -> dict | None:
    me = face_override(f, rotation, True)
    for o in overrides:
        if "normal" in o and same_override(me, o):
            return o
    return None


def face_override(f: Face, rotation: float, visible: bool) -> dict:
    """Override record (map frame, so it survives re-alignment) for toggling a face."""
    ax, ay = f.a
    bx, by = f.b
    mid = _rot(np.array([[(ax + bx) / 2, (ay + by) / 2]]), rotation)[0]
    normal = (f.angle + rotation + (f.side or 1) * math.pi / 2) % (2 * math.pi)
    return {"x": float(mid[0]), "y": float(mid[1]), "normal": float(normal), "visible": visible}


def map_rotation(grid: OccupancyGrid) -> float:
    pts, theta, _, strength = _surface_points(grid)
    return sharpen_angle(pts[strength > 0.3], dominant_angle(theta, strength))


def extract(
    grid: OccupancyGrid,
    areas: list[dict] | None = None,
    rotation: float | None = None,
    overrides: list[dict] | None = None,
) -> FloorPlan:
    pts, theta, normal, strength = _surface_points(grid)
    if rotation is None:
        rotation = sharpen_angle(pts[strength > 0.3], dominant_angle(theta, strength))
    gv = _GridView(grid, rotation)
    if len(pts) == 0:
        return FloorPlan(rotation, [], [], [], 0.0)
    pa = _rot(pts, -rotation)
    na = _rot(normal, -rotation)
    ta = np.mod(theta - rotation, math.pi)
    lin = strength

    # Wall directions: the two Manhattan axes plus any other well-supported angle.
    phis = [0.0, math.pi / 2]
    d0 = np.minimum(np.abs((ta + math.pi / 2) % math.pi - math.pi / 2), np.abs((ta - math.pi / 2 + math.pi / 2) % math.pi - math.pi / 2))
    other = (lin > 0.6) & (d0 > math.radians(15))
    if other.sum() >= 6:
        a = np.sort(ta[other])
        a_ext = np.concatenate([a - math.pi, a, a + math.pi])
        win = math.radians(3)
        counts = np.searchsorted(a_ext, a + win) - np.searchsorted(a_ext, a - win)
        for i in np.argsort(-counts):
            if counts[i] < 6:
                break
            near = np.abs((a - a[i] + math.pi / 2) % math.pi - math.pi / 2) <= win
            phi = float(np.angle(np.mean(np.exp(2j * a[near]))) / 2 % math.pi)
            snapped = round(phi / (math.pi / 4)) * (math.pi / 4)
            if abs(phi - snapped) < math.radians(3):
                phi = snapped % math.pi
            if all(abs((phi - p + math.pi / 2) % math.pi - math.pi / 2) > math.radians(10) for p in phis):
                phis.append(phi)

    faces: list[Face] = []
    for phi in phis:
        faces += _faces_for_angle(phi, pa, ta, na, strength, grid.res, len(faces))
    faces, doors = _merge_and_split(faces, gv)
    on_axis = lambda f: abs((f.angle + math.pi / 4) % (math.pi / 2) - math.pi / 4) < 1e-6  # noqa: E731
    faces = [f for f in faces if f.length >= 0.15 and (on_axis(f) or f.n >= 8)]
    _classify_furniture(faces, gv)
    _close_corners(faces, gv)
    def axis(f):
        return min(abs((f.angle + math.pi / 4) % (math.pi / 2) - math.pi / 4), 1.0) < 1e-6

    # Off-axis faces must be longer: rounded corners produce short diagonal runs.
    faces = [f for f in faces if f.length >= (MIN_FACE_M if axis(f) else 0.35) and (axis(f) or f.n >= 8)]
    for i, f in enumerate(faces):
        f.id = i
    _classify(faces, gv)
    for f in faces:
        o = _match_override(f, overrides or [], rotation)
        if o is not None:
            f.visible = bool(o["visible"])
            if f.visible and f.kind == "furniture":
                f.kind = "wall"
    _corner_fill(faces)
    doors = _dedupe_doors(doors, faces)
    for d in doors:
        _refine_door_width(d, pa)
    walls = [f for f in faces if f.visible and f.kind in ("wall", "exterior")]
    rooms = _room_labels(areas or [], gv, walls, rotation)

    area = _interior_area(gv, faces)

    bodies = [p for f in faces if f.visible for p in f.body()]
    if bodies:
        b = np.array(bodies)
        bounds = (float(b[:, 0].min()), float(b[:, 1].min()), float(b[:, 0].max()), float(b[:, 1].max()))
    else:
        bounds = (0.0, 0.0, 0.0, 0.0)
    stats = {
        "points": int(len(pts)),
        "faces": len(faces),
        "walls": len(walls),
        "furniture": sum(f.kind == "furniture" for f in faces),
        "fixtures": sum(f.kind == "fixture" for f in faces),
        "directions_deg": [round(math.degrees(p), 1) for p in phis],
        "occupied_cells": int((gv.cls == OCCUPIED).sum()),
    }
    return FloorPlan(rotation, faces, doors, rooms, area, bounds, stats)
