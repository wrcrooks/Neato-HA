from ..config import Config
from .base import MoveResult, Odometry, Robot, RobotError, RobotStatus, Safety, Scan


def make_robot(cfg: Config) -> Robot:
    if cfg.robot == "openneato":
        from .openneato import OpenNeatoRobot

        return OpenNeatoRobot(cfg)
    if cfg.robot == "sim":
        from .sim import SimRobot

        return SimRobot(cfg)
    raise ValueError(f"unknown robot backend {cfg.robot!r}")


__all__ = ["make_robot", "MoveResult", "Odometry", "Robot", "RobotError", "RobotStatus", "Safety", "Scan"]
