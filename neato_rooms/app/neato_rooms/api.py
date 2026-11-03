"""HTTP + WebSocket API and PWA hosting.

All URLs are relative so the app works behind Home Assistant ingress
(/api/hassio_ingress/<token>/...).
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .config import Config
from .geometry import Pose
from .robot import RobotError
from .service import Service

log = logging.getLogger(__name__)
WEB_DIR = Path(__file__).parent / "web"


class AreaIn(BaseModel):
    name: str | None = None
    kind: str = "room"
    polygon: list[list[float]] = Field(default_factory=list)
    color: str | None = None


class AreaPatch(BaseModel):
    name: str | None = None
    kind: str | None = None
    polygon: list[list[float]] | None = None
    color: str | None = None


class MissionIn(BaseModel):
    kind: str  # explore | manual_map | clean | dock | calibrate
    area_ids: list[str] = Field(default_factory=list)
    new_map: bool = True
    passes: int = 1


class DriveIn(BaseModel):
    cmd: str  # forward | rotate | finish
    amount: float = 0.0  # metres or radians (CCW positive)


class PoseIn(BaseModel):
    x: float
    y: float
    theta: float


def create_app(cfg: Config, service: Service | None = None) -> FastAPI:
    svc = service or Service(cfg)

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        await svc.start()
        yield
        await svc.stop()

    app = FastAPI(title="Neato Rooms", lifespan=lifespan)
    app.state.service = svc

    def bad(e: Exception, code: int = 400):
        raise HTTPException(code, str(e))

    async def areas_changed():
        await svc.broadcast({"type": "areas", "areas": svc.store.areas})

    @app.get("/api/state")
    async def state():
        return svc.snapshot()

    @app.get("/api/map")
    async def get_map():
        return svc.map_message()

    @app.get("/api/live")
    async def live():
        return svc.live()

    @app.get("/api/areas")
    async def list_areas():
        return svc.store.areas

    @app.post("/api/areas")
    async def add_area(a: AreaIn):
        try:
            area = svc.store.add_area(a.model_dump())
        except ValueError as e:
            bad(e)
        await areas_changed()
        return area

    @app.patch("/api/areas/{area_id}")
    async def patch_area(area_id: str, a: AreaPatch):
        try:
            area = svc.store.update_area(area_id, {k: v for k, v in a.model_dump().items() if v is not None})
        except KeyError:
            bad("no such area", 404)
        except ValueError as e:
            bad(e)
        await areas_changed()
        return area

    @app.delete("/api/areas/{area_id}")
    async def delete_area(area_id: str):
        try:
            svc.store.delete_area(area_id)
        except KeyError:
            bad("no such area", 404)
        await areas_changed()
        return {"ok": True}

    @app.post("/api/mission")
    async def start_mission(m: MissionIn):
        kw: dict = {}
        if m.kind == "clean":
            kw = {"area_ids": m.area_ids, "passes": max(1, min(m.passes, 3))}
        elif m.kind in ("explore", "manual_map"):
            kw = {"new_map": m.new_map}
        elif m.kind not in ("dock", "calibrate"):
            bad(f"unknown mission {m.kind}")
        try:
            svc.status = await svc.robot.status()
            info = svc.start_mission(m.kind, **kw)
        except RuntimeError as e:
            bad(e, 409)
        return info

    @app.post("/api/mission/cancel")
    async def cancel_mission():
        await svc.cancel()
        return {"ok": True}

    @app.post("/api/drive")
    async def drive(d: DriveIn):
        try:
            svc.drive(d.cmd, d.amount)
        except (RuntimeError, ValueError) as e:
            bad(e, 409)
        return {"ok": True}

    @app.post("/api/map/save")
    async def save_map():
        svc.save_map()
        return {"ok": True}

    @app.delete("/api/map")
    async def delete_map():
        try:
            svc.reset_map()
        except RuntimeError as e:
            bad(e, 409)
        await svc.broadcast(svc.map_message())
        return {"ok": True}

    @app.post("/api/pose")
    async def set_pose(p: PoseIn):
        svc.set_pose(Pose(p.x, p.y, p.theta))
        await svc.broadcast(svc.live())
        return {"ok": True}

    @app.post("/api/native/{action}")
    async def native(action: str):
        if action not in ("house", "spot", "pause", "stop", "dock"):
            bad("unknown action", 404)
        try:
            await svc.native(action)
        except (RuntimeError, RobotError) as e:
            bad(e, 409)
        return {"ok": True}

    @app.websocket("/api/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        q = svc.subscribe()
        try:
            await sock.send_json({"type": "status", **svc.snapshot()})
            await sock.send_json(svc.map_message())
            await sock.send_json({"type": "areas", "areas": svc.store.areas})
            await sock.send_json(svc.live())
            while True:
                msg = await q.get()
                await sock.send_json(msg)
        except (WebSocketDisconnect, RuntimeError, asyncio.CancelledError):
            pass
        finally:
            svc.unsubscribe(q)

    @app.get("/")
    async def index():
        return FileResponse(WEB_DIR / "index.html", headers={"Cache-Control": "no-cache"})

    app.mount("/", StaticFiles(directory=WEB_DIR), name="web")
    return app
