"""Robot backed by an ESP32 running OpenNeato (github.com/renjfk/OpenNeato).

OpenNeato keeps the safety loop on the ESP32: in manual mode it polls bumpers,
wheel-lift and wheel stall every 500 ms and stops the wheels itself, and it stops
the wheels if no API request arrives for 5 s. This client only adds waiting for
a move to finish and reading back what happened.
"""

from __future__ import annotations

import asyncio
import logging
import time

import httpx
import numpy as np

from ..config import Config
from .base import MoveResult, Odometry, Robot, RobotError, RobotStatus, Safety, Scan

log = logging.getLogger(__name__)

POLL_S = 0.15
SETTLE_POLLS = 3  # consecutive unchanged odometry reads => wheels stopped
ARRIVE_TOL_MM = 6.0


class OpenNeatoRobot(Robot):
    def __init__(self, cfg: Config, client: httpx.AsyncClient | None = None):
        super().__init__(cfg)
        self._http = client or httpx.AsyncClient(base_url=cfg.openneato_url.rstrip("/"), timeout=10.0)
        # The ESP32 serialises serial access; avoid piling up concurrent requests.
        self._lock = asyncio.Lock()
        self._manual = False

    async def close(self) -> None:
        await self._http.aclose()

    async def _request(self, method: str, path: str, **params) -> httpx.Response:
        async with self._lock:
            for attempt in range(3):
                try:
                    r = await self._http.request(method, path, params=params or None)
                except httpx.HTTPError as e:
                    if attempt == 2:
                        raise RobotError(f"{method} {path}: {e}") from e
                    await asyncio.sleep(0.3)
                    continue
                if r.status_code == 503 and attempt < 2:  # serial queue busy
                    await asyncio.sleep(0.3)
                    continue
                return r
        raise RobotError(f"{method} {path}: unreachable")  # pragma: no cover

    async def _get(self, path: str) -> dict:
        r = await self._request("GET", path)
        if r.status_code != 200:
            raise RobotError(f"GET {path}: HTTP {r.status_code} {r.text[:200]}")
        return r.json()

    async def _post(self, path: str, **params) -> bool:
        r = await self._request("POST", path, **params)
        return r.status_code == 200

    # -- Robot API --------------------------------------------------------------

    async def status(self) -> RobotStatus:
        st = RobotStatus(manual=self._manual)
        try:
            charger = await self._get("/api/charger")
            state = await self._get("/api/state")
            err = await self._get("/api/error")
        except RobotError as e:
            st.error = str(e)
            return st
        st.connected = True
        st.battery = charger.get("fuelPercent")
        st.charging = bool(charger.get("chargingActive"))
        st.on_dock = bool(charger.get("extPwrPresent"))
        st.state = state.get("robotState") or state.get("uiState") or "unknown"
        if err.get("hasError"):
            st.error = err.get("displayMessage") or err.get("errorMessage")
        return st

    async def set_manual(self, enable: bool) -> None:
        if enable == self._manual:
            return
        ok = await self._post("/api/manual", enable=1 if enable else 0)
        if not ok and enable:
            raise RobotError("OpenNeato refused to enter manual mode (robot busy or in error?)")
        self._manual = enable
        if enable:
            await asyncio.sleep(1.5)  # let the LIDAR spin up to speed

    async def odometry(self) -> Odometry:
        m = await self._get("/api/motors")
        return Odometry(float(m["leftWheelPositionMM"]), float(m["rightWheelPositionMM"]))

    async def scan(self) -> Scan:
        data = await self._get("/api/lidar")
        pts = data.get("points", [])
        return Scan(
            angle_deg=np.array([p["angle"] for p in pts], dtype=int),
            dist_mm=np.array([p["dist"] for p in pts], dtype=int),
            error=np.array([p["error"] for p in pts], dtype=int),
        )

    async def _safety(self) -> Safety:
        s = await self._get("/api/manual/status")
        return Safety(
            bumper=any(s.get(k) for k in ("bumperFrontLeft", "bumperFrontRight", "bumperSideLeft", "bumperSideRight")),
            lifted=bool(s.get("lifted")),
            stalled=bool(s.get("stallFront") or s.get("stallRear")),
        )

    async def move(self, left_mm: float, right_mm: float, speed_mm_s: int) -> MoveResult:
        if not self._manual:
            raise RobotError("move() requires manual mode")
        start = await self.odometry()
        left_mm, right_mm = round(left_mm), round(right_mm)
        if left_mm == 0 and right_mm == 0:
            return MoveResult(0, 0)
        accepted = await self._post("/api/manual/move", left=left_mm, right=right_mm, speed=int(speed_mm_s))
        if not accepted:
            # OpenNeato blocks moves into an active bumper / while lifted.
            return MoveResult(0, 0, await self._safety())

        expected_s = max(abs(left_mm), abs(right_mm)) / max(speed_mm_s, 1)
        deadline = time.monotonic() + expected_s * 2.0 + 3.0
        last = start
        unchanged = 0
        moved = False
        safety = Safety()
        polls = 0
        while time.monotonic() < deadline:
            await asyncio.sleep(POLL_S)
            cur = await self.odometry()
            dl, dr = cur.left_mm - start.left_mm, cur.right_mm - start.right_mm
            if abs(dl - left_mm) <= ARRIVE_TOL_MM and abs(dr - right_mm) <= ARRIVE_TOL_MM:
                break
            if (cur.left_mm, cur.right_mm) == (last.left_mm, last.right_mm):
                unchanged += 1
            else:
                unchanged = 0
                moved = True
            last = cur
            polls += 1
            if polls % 3 == 0:
                safety = await self._safety()
                if safety.any:
                    break
            if unchanged >= SETTLE_POLLS and (moved or polls > 8):
                break
        else:
            log.warning("move(%s,%s) timed out; stopping wheels", left_mm, right_mm)
            await self.stop()
        # Wait for any coasting to finish, then read final odometry.
        await asyncio.sleep(POLL_S)
        end = await self.odometry()
        if not safety.any:
            safety = await self._safety()
        return MoveResult(end.left_mm - start.left_mm, end.right_mm - start.right_mm, safety)

    async def stop(self) -> None:
        await self._post("/api/manual/move", left=0, right=0, speed=0)

    async def set_cleaning(self, brush: bool, vacuum: bool, side_brush: bool) -> None:
        await self._post(
            "/api/manual/motors", brush=int(brush), vacuum=int(vacuum), sideBrush=int(side_brush)
        )

    async def native(self, action: str) -> None:
        if action not in ("house", "spot", "pause", "stop", "dock"):
            raise ValueError(action)
        if self._manual:
            await self.set_manual(False)
        if not await self._post("/api/clean", action=action):
            raise RobotError(f"robot rejected '{action}'")
