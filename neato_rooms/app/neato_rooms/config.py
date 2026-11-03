"""Runtime configuration.

Values come from (in increasing priority): defaults, the Home Assistant add-on
options file (/data/options.json), then NEATO_* environment variables.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, fields
from pathlib import Path


@dataclass
class Config:
    # "openneato" talks to the ESP32 bridge; "sim" runs the built-in simulator.
    robot: str = "sim"
    openneato_url: str = "http://neato.local"
    data_dir: str = "/data"
    host: str = "0.0.0.0"
    port: int = 8099

    # Robot geometry. UNVERIFIED on real hardware: calibrate (see PLAN.md §6).
    wheel_base_mm: float = 241.0
    robot_radius_m: float = 0.17
    lidar_offset_x_mm: float = -90.0
    lidar_angle_offset_deg: float = 0.0
    lidar_ccw: bool = True
    lidar_max_range_m: float = 5.0
    lidar_min_range_m: float = 0.12

    # Motion.
    speed_mm_s: int = 200
    max_step_m: float = 0.35

    # Mapping / cleaning.
    map_resolution_m: float = 0.05
    map_size_m: float = 40.0
    lane_spacing_m: float = 0.24
    safety_margin_m: float = 0.03

    # Simulator only: seconds of wall-clock per simulated second (0 = instant).
    sim_time_scale: float = 0.25

    @classmethod
    def load(cls, options_path: str | None = None) -> "Config":
        cfg = cls()
        path = Path(options_path or os.environ.get("NEATO_OPTIONS", "/data/options.json"))
        if path.is_file():
            cfg._apply(json.loads(path.read_text()))
        env = {}
        for f in fields(cls):
            key = "NEATO_" + f.name.upper()
            if key in os.environ:
                env[f.name] = os.environ[key]
        cfg._apply(env)
        return cfg

    def _apply(self, values: dict) -> None:
        for f in fields(self):
            if f.name not in values or values[f.name] is None:
                continue
            raw = values[f.name]
            current = getattr(self, f.name)
            if isinstance(current, bool):
                val = raw if isinstance(raw, bool) else str(raw).lower() in ("1", "true", "yes", "on")
            elif isinstance(current, int):
                val = int(raw)
            elif isinstance(current, float):
                val = float(raw)
            else:
                val = str(raw)
            setattr(self, f.name, val)
