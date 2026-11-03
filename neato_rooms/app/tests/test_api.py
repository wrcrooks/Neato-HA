from fastapi.testclient import TestClient

from neato_rooms.api import create_app
from neato_rooms.config import Config


def test_api_smoke(tmp_path):
    cfg = Config(sim_time_scale=0.02, data_dir=str(tmp_path))
    with TestClient(create_app(cfg)) as c:
        assert c.get("/").status_code == 200
        assert c.get("/manifest.webmanifest").status_code == 200
        st = c.get("/api/state").json()
        assert st["backend"] == "sim" and st["has_map"] is False

        a = c.post("/api/areas", json={"name": "Hall", "kind": "room", "polygon": [[0, 0], [1, 0], [1, 1]]}).json()
        assert a["id"] and a["color"]
        assert c.post("/api/areas", json={"kind": "room", "polygon": [[0, 0]]}).status_code == 400
        assert c.patch(f"/api/areas/{a['id']}", json={"name": "Hallway"}).json()["name"] == "Hallway"
        assert c.post("/api/areas", json={"kind": "bogus", "polygon": [[0, 0], [1, 0], [1, 1]]}).status_code == 400

        with c.websocket_connect("/api/ws") as ws:
            kinds = {ws.receive_json()["type"] for _ in range(4)}
            assert kinds == {"status", "map", "areas", "live"}

        assert c.post("/api/drive", json={"cmd": "forward"}).status_code == 409  # not mapping
        assert c.post("/api/mission", json={"kind": "manual_map"}).status_code == 200
        assert c.post("/api/mission", json={"kind": "explore"}).status_code == 409  # busy
        assert c.post("/api/mission/cancel").status_code == 200
        assert c.get("/api/state").json()["mission"]["status"] in ("cancelled", "done")

        assert c.delete(f"/api/areas/{a['id']}").status_code == 200
        assert c.delete(f"/api/areas/{a['id']}").status_code == 404
