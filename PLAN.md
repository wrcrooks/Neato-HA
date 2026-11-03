# Neato D7 Revival — Plan

Goal: after the Neato cloud shutdown (Oct 6 2025), get back **house mapping, labelled
rooms, and "clean this room / zone"** on a Botvac D7, fully local, from a PWA.

## 1. Architecture

```
 ┌───────────── Botvac D7 ─────────────┐        WiFi         ┌──────── Home Assistant host ────────┐
 │ mainboard ── 4-pin debug UART ──┐   │                     │  "Neato Rooms" add-on (this repo)   │
 │  (LIDAR, wheels, brush, vacuum) │   │   HTTP (LAN)        │  ┌───────────────────────────────┐  │
 │                         ESP32 + OpenNeato  ◄──────────────►│  │ server/ (Python, FastAPI)     │  │
 │                         (serial→HTTP, on-device safety)    │  │  • robot client (OpenNeato)   │  │
 └─────────────────────────────────────┘                     │  │  • SLAM: occupancy grid +     │  │
                                                             │  │    correlative scan matching  │  │
                         Phone / browser ◄── HA ingress ────►│  │  • A* planner + coverage      │  │
                         (PWA: map, rooms, clean buttons)    │  │  • mission executor           │  │
                                                             │  │  • REST + WebSocket, PWA      │  │
                                                             │  └───────────────────────────────┘  │
                                                             └─────────────────────────────────────┘
```

Why this split:

* **OpenNeato** (MIT, github.com/renjfk/OpenNeato) already exposes the D7's serial port over
  HTTP: `/api/lidar`, `/api/motors` (wheel odometry), `/api/manual` (TestMode + LIDAR spin),
  `/api/manual/move` (distance-limited wheel moves), `/api/manual/motors` (brush/vacuum/side
  brush), `/api/manual/status` (bumpers, wheel-lift, stall), `/api/clean?action=dock|house|…`.
  Crucially its **safety loop runs on the ESP32** (bumper/lift/stall polling every 500 ms and a
  5 s client watchdog), so if WiFi or the server drops, the robot stops.
* The heavy lifting (SLAM, planning, UI) needs more CPU/RAM than an ESP32 has, so it runs on the
  Home Assistant box as an add-on, with the PWA served through HA ingress.
* No existing project (OpenNeato, vacuula/fang) does rooms/zones yet, and the D7 firmware has no
  known serial command that accepts a zone — so we drive the robot ourselves in TestMode.

## 2. How the robot is driven

* **Stop-and-scan.** Every move is a short distance-limited command (≤ 0.35 m straight or an
  in-place rotation). After each move the robot is stationary when we pull a LIDAR scan, so scans
  have no motion skew. Pose = odometry prediction refined by scan matching.
* Distance-limited moves (`SetMotor LWheelDist RWheelDist Speed`) mean a lost connection can never
  cause a runaway — the robot stops after the commanded distance.
* Map frame: origin = robot pose on the dock when mapping started; x forward, y left, θ CCW.

## 3. Features / flows

1. **Map the house** — start from the dock; either *auto-explore* (frontier exploration) or drive
   with the PWA joystick. The map builds live in the PWA. Save when done.
2. **Rooms & zones** — draw polygons (rooms) or rectangles (zones) on the map, name them, and
   optionally draw no-go areas.
3. **Clean** — pick one or more rooms/zones → the server undocks, relocalises against the saved
   map, A*-navigates to each area, runs a boustrophedon (lawn-mower) coverage path with brush +
   vacuum on, then returns to the dock pose and hands over to the robot's native docking.
4. **Native fallbacks** — "Clean house", "Spot", "Dock" buttons pass straight through to the
   robot's own firmware.

## 4. Repo layout

See README.md. In short: `neato_rooms/` is the Home Assistant add-on, and the Python app
(SLAM, planning, API, PWA) lives in `neato_rooms/app/`.

## 5. Phases

| Phase | What | Status |
|---|---|---|
| 0 | Hardware: open D7, wire ESP32 to debug header, flash OpenNeato, confirm `http://neato.local` works | **you** |
| 1 | Server skeleton, robot interface, simulator, OpenNeato client | ✅ done (sim + mocked OpenNeato tests) |
| 2 | SLAM (grid + scan matching) + mapping missions (manual + auto-explore) | ✅ done in sim |
| 3 | PWA: live map, D-pad, room/zone/no-go editor | ✅ done |
| 4 | Planner, coverage, clean missions, return-to-dock | ✅ done in sim (~96% room coverage) |
| 5 | HA add-on packaging | ✅ image builds and runs |
| 6 | **On-robot calibration & tuning** (see §6) | together, once hardware is wired |
| 7 | Later: schedules per room, HA entities/services, use robot's own `GetRobotPos` pose during native cleans | later |

## 6. Hardware facts still to verify on your robot

These are configurable in the add-on options and **must be checked on the real D7**:

* `wheel_base_mm` (default 241). Measure it with the built-in calibration (Map tab).
* LIDAR mounting: `lidar_offset_x_mm` (default −90, assuming the LDS sits behind the wheel
  axle) and angle offset (default 0° = forward). Calibration checks the angle direction
  (default CCW, matching OpenNeato's UI).
* Lane spacing (default 0.24 m) vs. the D7's brush width.
* That OpenNeato's `dock` action (`SEND_TO_BASE`) docks reliably when started ~0.5 m in front of
  the base. If not, we'll add a final docking approach using the base's LIDAR signature.
* Safe speeds on your floors (default 200 mm/s; firmware max 300).
* **Cliff/drop sensors.** OpenNeato's manual mode doesn't check them. Read the field names from
  `POST /api/serial?cmd=GetAnalogSensors` on the real robot, then add a drop check to every
  forward step. Until then, stairs must be blocked and drawn as no-go.
* LIDAR scan time over WiFi. Exploring a flat takes ~600 stop-and-scan steps, so expect
  roughly 20–30 min if each step takes 2–3 s.

## 7. Phase 0 — hardware steps (summary)

1. Buy an ESP32-S3 or ESP32-C3 dev board with **4 MB flash**, and a T10 **security** Torx bit.
2. Follow OpenNeato's user guide + the neato-brainslug teardown
   (github.com/Philip2809/neato-brainslug) to reach the 4-pin debug header:
   `RX, 3V3, TX, GND` (left→right, robot labels). Cross RX/TX.
3. Flash with `openneato-flash`, set WiFi, open `http://neato.local`, confirm LIDAR + manual
   drive work in OpenNeato's own UI before using this add-on.
