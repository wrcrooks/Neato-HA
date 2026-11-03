"""OpenNeatoRobot against a fake OpenNeato HTTP API (request shapes match OpenNeato's frontend/api.ts)."""

import httpx
import pytest

from neato_rooms.config import Config
from neato_rooms.robot.base import RobotError
from neato_rooms.robot.openneato import OpenNeatoRobot


class FakeOpenNeato:
    """Wheels advance a fixed amount per /api/motors poll, like a real robot moving."""

    def __init__(self, bump_after_mm: float | None = None):
        self.manual = False
        self.pos = [1000.0, 2000.0]
        self.target = list(self.pos)
        self.step = 40.0
        self.bump_after_mm = bump_after_mm
        self.bumper = False
        self.moved_since_cmd = 0.0
        self.log: list[str] = []

    def handler(self, req: httpx.Request) -> httpx.Response:
        path, q = req.url.path, dict(req.url.params)
        self.log.append(f"{req.method} {path} {q}")
        if path == "/api/manual" and req.method == "POST":
            self.manual = q["enable"] == "1"
            return httpx.Response(200, json={"ok": True})
        if path == "/api/manual/move":
            if not self.manual:
                return httpx.Response(503, json={"error": "unavailable"})
            left, right = int(q["left"]), int(q["right"])
            if self.bumper and left > 0 and right > 0:
                return httpx.Response(500, json={"error": "blocked"})
            self.target = [self.pos[0] + left, self.pos[1] + right]
            self.moved_since_cmd = 0.0
            return httpx.Response(200, json={"ok": True})
        if path == "/api/motors":
            for i in range(2):
                d = self.target[i] - self.pos[i]
                self.pos[i] += max(-self.step, min(self.step, d))
            self.moved_since_cmd += self.step
            if self.bump_after_mm and self.moved_since_cmd >= self.bump_after_mm and self.target != self.pos:
                self.bumper = True
                self.target = list(self.pos)  # OpenNeato stops the wheels on contact
            return httpx.Response(200, json={"leftWheelPositionMM": int(self.pos[0]), "rightWheelPositionMM": int(self.pos[1])})
        if path == "/api/manual/status":
            return httpx.Response(200, json={"active": self.manual, "bumperFrontLeft": self.bumper, "lifted": False})
        if path == "/api/lidar":
            pts = [{"angle": a, "dist": 1500 if a % 2 else 0, "intensity": 100, "error": 0 if a % 2 else 0x8035} for a in range(360)]
            return httpx.Response(200, json={"rotationSpeed": 5.0, "validPoints": 180, "points": pts})
        if path == "/api/charger":
            return httpx.Response(200, json={"fuelPercent": 87, "chargingActive": False, "extPwrPresent": True})
        if path == "/api/state":
            return httpx.Response(200, json={"uiState": "UIMGR_STATE_IDLE", "robotState": "ST_C_Standby"})
        if path == "/api/error":
            return httpx.Response(200, json={"hasError": False, "errorCode": 200})
        if path == "/api/clean":
            return httpx.Response(200, json={"ok": True})
        return httpx.Response(404)


def make(fake: FakeOpenNeato) -> OpenNeatoRobot:
    cfg = Config(robot="openneato")
    client = httpx.AsyncClient(base_url="http://neato.test", transport=httpx.MockTransport(fake.handler))
    return OpenNeatoRobot(cfg, client=client)


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    import neato_rooms.robot.openneato as on

    monkeypatch.setattr(on, "POLL_S", 0.0)

    async def no_sleep(_):
        return None

    monkeypatch.setattr(on.asyncio, "sleep", no_sleep)


async def test_status_and_scan():
    fake = FakeOpenNeato()
    r = make(fake)
    st = await r.status()
    assert st.connected and st.battery == 87 and st.on_dock and st.state == "ST_C_Standby"
    await r.set_manual(True)
    scan = await r.scan()
    pts = scan.to_points(r.cfg)
    assert len(pts) == 180  # invalid (error / zero) readings dropped


async def test_move_waits_for_completion():
    fake = FakeOpenNeato()
    r = make(fake)
    with pytest.raises(RobotError):
        await r.move(100, 100, 200)  # not in manual mode
    await r.set_manual(True)
    res = await r.move(300, 300, 200)
    assert res.left_mm == pytest.approx(300) and res.right_mm == pytest.approx(300)
    assert not res.safety.any
    res = await r.rotate(3.14159 / 2, 120)
    assert res.left_mm == pytest.approx(-189, abs=1) and res.right_mm == pytest.approx(189, abs=1)
    assert any("left=300&right=300&speed=200" in line or "'left': '300'" in line for line in fake.log)


async def test_move_reports_bump():
    fake = FakeOpenNeato(bump_after_mm=120)
    r = make(fake)
    await r.set_manual(True)
    res = await r.move(500, 500, 200)
    assert res.safety.bumper
    assert res.left_mm < 500
    # Forward is now refused while the bumper is pressed; reversing still works.
    res2 = await r.move(100, 100, 200)
    assert res2.left_mm == 0 and res2.safety.bumper
    fake.bumper = False
    back = await r.move(-80, -80, 100)
    assert back.left_mm == pytest.approx(-80)


async def test_native_leaves_manual_mode():
    fake = FakeOpenNeato()
    r = make(fake)
    await r.set_manual(True)
    await r.native("dock")
    assert not fake.manual
    assert any("/api/clean" in line and "dock" in line for line in fake.log)
