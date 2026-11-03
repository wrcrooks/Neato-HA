"""Robot interface shared by the OpenNeato client and the simulator."""

from __future__ import annotations

import abc
from dataclasses import dataclass, field

import numpy as np

from ..config import Config


@dataclass
class Scan:
    """One LIDAR revolution, as reported by the robot (one reading per degree)."""

    angle_deg: np.ndarray  # (N,) int
    dist_mm: np.ndarray  # (N,) int, 0 = no return
    error: np.ndarray  # (N,) int, non-zero = invalid

    def to_points(self, cfg: Config) -> np.ndarray:
        """Valid returns as (M,2) points in the robot frame (metres, centred on the wheel axle)."""
        d = self.dist_mm.astype(float) / 1000.0
        ok = (self.error == 0) & (d >= cfg.lidar_min_range_m) & (d <= cfg.lidar_max_range_m)
        sign = 1.0 if cfg.lidar_ccw else -1.0
        a = np.radians(sign * self.angle_deg[ok].astype(float) + cfg.lidar_angle_offset_deg)
        pts = np.stack([d[ok] * np.cos(a), d[ok] * np.sin(a)], axis=1)
        pts[:, 0] += cfg.lidar_offset_x_mm / 1000.0
        return pts

    def as_list(self) -> list[list[int]]:
        return [[int(a), int(d)] for a, d, e in zip(self.angle_deg, self.dist_mm, self.error) if e == 0 and d > 0]


@dataclass
class Odometry:
    left_mm: float
    right_mm: float


@dataclass
class Safety:
    bumper: bool = False
    lifted: bool = False
    stalled: bool = False

    @property
    def any(self) -> bool:
        return self.bumper or self.lifted or self.stalled


@dataclass
class MoveResult:
    left_mm: float  # wheel travel actually achieved
    right_mm: float
    safety: Safety = field(default_factory=Safety)


@dataclass
class RobotStatus:
    connected: bool = False
    battery: int | None = None
    charging: bool = False
    on_dock: bool = False
    state: str = "unknown"
    error: str | None = None
    manual: bool = False

    def as_dict(self) -> dict:
        return self.__dict__.copy()


class RobotError(RuntimeError):
    pass


class Robot(abc.ABC):
    """Minimal set of operations the mapper and missions need."""

    def __init__(self, cfg: Config):
        self.cfg = cfg

    async def close(self) -> None:  # pragma: no cover - default no-op
        pass

    @abc.abstractmethod
    async def status(self) -> RobotStatus: ...

    @abc.abstractmethod
    async def set_manual(self, enable: bool) -> None:
        """Enter/leave direct-drive mode (TestMode + LIDAR spinning on OpenNeato)."""

    @abc.abstractmethod
    async def odometry(self) -> Odometry: ...

    @abc.abstractmethod
    async def scan(self) -> Scan: ...

    @abc.abstractmethod
    async def move(self, left_mm: float, right_mm: float, speed_mm_s: int) -> MoveResult:
        """Distance-limited wheel move. Returns once the wheels have stopped."""

    @abc.abstractmethod
    async def set_cleaning(self, brush: bool, vacuum: bool, side_brush: bool) -> None: ...

    @abc.abstractmethod
    async def native(self, action: str) -> None:
        """Pass-through to the robot's firmware: house | spot | pause | stop | dock."""

    async def keepalive(self) -> None:
        """Called periodically while idle in manual mode (OpenNeato has a 5 s watchdog)."""
        await self.status()

    # Convenience ---------------------------------------------------------------

    async def rotate(self, angle_rad: float, speed_mm_s: int) -> MoveResult:
        arc = angle_rad * self.cfg.wheel_base_mm / 2.0
        return await self.move(-arc, arc, speed_mm_s)

    async def forward(self, dist_m: float, speed_mm_s: int) -> MoveResult:
        d = dist_m * 1000.0
        return await self.move(d, d, speed_mm_s)

