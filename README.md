# Neato D7 Revival — Neato Rooms

The Neato cloud shut down on Oct 6, 2025, and took the app, maps, zones and room cleaning with
it. This project brings those back for Botvac **D3–D7**, fully local:

* **Map** the house (auto-explore, or drive it yourself), in as many runs as you like: runs that
  start away from the dock are **stitched** on, and a **re-scan** sharpens the map
* **Review what changed** after each new scan (furniture added, removed or moved, newly mapped
  area), keep or ignore each change, and preview the floor plan before applying it
* **Label rooms**, draw cleaning **zones** and **no-go** areas
* **Clean selected rooms/zones**, then return to the dock
* A **floor plan** in the style of a real-estate listing: wall thicknesses, doors with swings,
  room dimensions and total area, aligned to the walls, exported as **SVG or PDF** at a standard
  scale
* A **PWA** UI, served as a **Home Assistant add-on** (sidebar panel)

```
D7 mainboard ─UART─ ESP32 (OpenNeato) ─WiFi/HTTP─ Home Assistant add-on (this repo) ─ PWA
```

[OpenNeato](https://github.com/renjfk/OpenNeato) turns the robot's serial port into an HTTP API
and runs the bumper/lift/stall safety loop on the ESP32. This add-on does mapping (SLAM), path
planning, coverage and the UI. See [PLAN.md](PLAN.md) for the design and roadmap, and
[neato_rooms/DOCS.md](neato_rooms/DOCS.md) for user docs. **Read the stairs warning there.**

## Status

* Works end-to-end against the built-in **simulator**: exploring maps all rooms, a selected
  room gets ~96% coverage, and the robot docks again.
* Floor plans from simulated maps get room sizes within a few cm, doors within ~5 cm and
  total area within ~1%, at any dock angle.
* A separately mapped run is stitched onto the map to within a few cm, and a moved sofa and a
  new armchair are reported as exactly those two changes.
* **Not yet run on a real D7.** Hardware-dependent values (wheel base, LIDAR mounting/direction,
  docking hand-off) are configurable, and there is a calibration routine for them. See PLAN.md §6.

## Install (Home Assistant)

1. This repo is the add-on repository: https://github.com/wrcrooks/Neato-HA
2. In Home Assistant, go to **Settings → Add-ons → Add-on store → ⋮ → Repositories** and add `https://github.com/wrcrooks/Neato-HA`.
3. Install **Neato Rooms**, set `openneato_url`, start it, and open **Neato Rooms** in the sidebar.

## Development

```bash
python3 -m venv .venv && .venv/bin/pip install -e 'neato_rooms/app[test]'
.venv/bin/pytest neato_rooms/app                            # unit + simulator end-to-end tests
NEATO_DATA_DIR=./data NEATO_ROBOT=sim .venv/bin/neato-rooms  # http://localhost:8099
NEATO_DATA_DIR=./data NEATO_ROBOT=openneato NEATO_OPENNEATO_URL=http://192.168.1.50 .venv/bin/neato-rooms
```

Any option in `neato_rooms/config.yaml` can be set as `NEATO_<OPTION>` (e.g. `NEATO_SIM_TIME_SCALE=0`
for an instant simulator).

Layout:

```
repository.yaml              Home Assistant add-on repository manifest
neato_rooms/                 the add-on (config.yaml, Dockerfile, DOCS.md)
  app/neato_rooms/
    robot/                   Robot interface · OpenNeato HTTP client · simulator
    slam/                    occupancy grid · correlative scan matcher · mapper
    nav/                     costmap · A* · frontier exploration · coverage lanes
    navigator.py             stop-and-scan motion, bump recovery, replanning
    service.py               missions (explore / manual map / clean / dock / calibrate)
    calibrate.py             wheel base + LIDAR direction from spin tests
    api.py                   REST + WebSocket, hosts the PWA
    web/                     PWA (vanilla JS, no build step)
  app/tests/
```

## License

MIT. OpenNeato is a separate MIT-licensed project. No code is copied from it; this add-on only
calls its HTTP API.
