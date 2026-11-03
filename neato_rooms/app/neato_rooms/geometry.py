"""2-D pose math. Map frame: metres, x forward from the dock, y left, theta CCW radians."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def compose(self, d: "Pose") -> "Pose":
        """Apply a motion `d` expressed in this pose's local frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose(self.x + c * d.x - s * d.y, self.y + s * d.x + c * d.y, wrap_angle(self.theta + d.theta))

    def transform_points(self, pts: np.ndarray) -> np.ndarray:
        """Robot-frame (N,2) points -> map frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        rot = np.array([[c, -s], [s, c]])
        return pts @ rot.T + np.array([self.x, self.y])

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.x, y - self.y)

    def as_dict(self) -> dict:
        return {"x": round(self.x, 4), "y": round(self.y, 4), "theta": round(self.theta, 4)}


def odometry_delta(dl_m: float, dr_m: float, wheel_base_m: float) -> Pose:
    """Differential-drive motion (local frame) from left/right wheel travel."""
    ds = (dl_m + dr_m) / 2.0
    dth = (dr_m - dl_m) / wheel_base_m
    if abs(dth) < 1e-9:
        return Pose(ds, 0.0, 0.0)
    # Arc motion: chord in the direction of the mid-heading.
    r = ds / dth
    return Pose(r * math.sin(dth), r * (1 - math.cos(dth)), wrap_angle(dth))


def point_in_polygon(x: np.ndarray, y: np.ndarray, poly: list[list[float]]) -> np.ndarray:
    """Vectorised even-odd point-in-polygon test."""
    inside = np.zeros(np.shape(x), dtype=bool)
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cond = (yi > y) != (yj > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= cond & (x < xint)
        j = i
    return inside
