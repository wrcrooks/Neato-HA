# Neato Rooms

Map your home, label rooms, and send a Neato Botvac D3–D7 to clean a single room or zone —
fully local, now that the Neato cloud is gone.

## Requirements

An ESP32 wired to the robot's internal debug port, running
[OpenNeato](https://github.com/renjfk/OpenNeato). Check that OpenNeato's own web UI works
(including **Manual** driving and the LIDAR view) before installing this add-on.

## ⚠️ Safety: stairs

Room cleaning drives the robot in OpenNeato's *manual mode* (the robot's TestMode). OpenNeato
stops the wheels on **bumper contact, wheel lift and wheel stall**, but the robot's **cliff
(drop) sensors are not monitored in this mode**, and the LIDAR cannot see a staircase.

If your floor has stairs or drop-offs:

* Block them physically while mapping and cleaning, **and**
* draw a **No-go** area over them once the map exists.

## Configuration

| Option | Default | What it does |
|---|---|---|
| `robot` | `openneato` | `sim` runs a built-in simulated robot and house, handy for trying the UI |
| `openneato_url` | `http://neato.local` | Address of the ESP32. Use its IP if `.local` names don't resolve on your network |
| `wheel_base_mm` | `241` | Distance between the wheels. **Run calibration** (Map tab) and use the suggested value |
| `lidar_ccw` | `true` | LIDAR angle direction. Calibration checks this |
| `lidar_offset_x_mm` | `-90` | LIDAR position relative to the wheel axle (negative = behind) |
| `lidar_angle_offset_deg` | `0` | Rotates LIDAR readings if 0° isn't straight ahead |
| `speed_mm_s` | `200` | Driving speed (max 300) |
| `lane_spacing_m` | `0.24` | Distance between cleaning lanes. Smaller = more overlap |

## First run

1. **Calibrate.** Put the robot on open floor about 1 m from walls. Open **Map → Calibrate → Run
   calibration**. Copy `wheel_base_mm_suggested` (and `lidar_ccw` if it differs) into the
   configuration and restart the add-on.
2. **Map.** Put the robot on its dock. **Map → Auto-explore** lets it find its own way. **Drive it
   myself** gives you arrow buttons. The dock becomes the map origin. The map saves when it
   finishes.
3. **Rooms.** Open **Rooms → + Room** and tap the corners of each room. **+ Zone** and
   **+ No-go** are rectangles you drag.
4. **Clean.** Tap rooms on the map (or in the list) → **Clean selected**. The robot leaves the
   dock, cleans each area in lanes, comes back, and hands over to its own docking routine.

The robot's own **Whole house** and **Spot** cleaning are still under *Robot's own cleaning*.

## How it works

The robot is driven in short stop-and-scan steps: move ≤ 35 cm, stop, read a LIDAR scan, and
correct the position by matching the scan against the map. Every wheel command is
distance-limited, and OpenNeato stops the wheels if it hears nothing for 5 s, so a dropped
connection can't send the robot running.

## Using it as a phone app

The sidebar panel works everywhere Home Assistant does. To install it as a standalone
app on your phone's home screen, enable the add-on's port 8099 (*Network* settings) and open
`http://<home-assistant>:8099`. Anyone on your network can then control the robot, and
browsers only allow full offline/app features over HTTPS.
