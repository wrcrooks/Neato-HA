# Neato Rooms

Map your home, label rooms, and send a Neato Botvac D3–D7 to clean a single room or zone —
fully local, now that the Neato cloud is gone.

## Requirements

An ESP32 wired to the robot's internal debug port, running
[OpenNeato](https://github.com/renjfk/OpenNeato). Check that OpenNeato's own web UI works
(including **Manual** driving and the LIDAR view) before installing this add-on.

### Which ESP32

OpenNeato needs an **ESP32-C3, ESP32-S3 or original ESP32 with at least 4 MB of flash**. 2 MB
boards won't work, because the firmware keeps two 1.6 MB update slots. Other chips, such as
the C6, H2 and S2, aren't supported, even though they come in tiny boards.

The board has to fit under the robot's mainboard, so small is better:

| Board | Size (approx.) | Notes |
|---|---|---|
| **ESP32-C3 Super Mini** (recommended) | 22.5 × 18 mm | The board OpenNeato's author uses. Cheapest. Buy it **without pre-soldered pins** |
| Seeed XIAO ESP32-C3 | 21 × 17.5 mm | Same chip, with an **external antenna** you can route out from under the mainboard. Pick this if Wi-Fi is weak where the robot works |
| Seeed XIAO ESP32-S3 | 21 × 17.5 mm | Also works. External antenna too. More flash than needed |

A bare ESP32-C3 module is smaller still, but you would have to add your own USB flashing setup.
It's not worth the hassle.

### Wiring

The D7's debug port is a **4-pin JST XH (2.54 mm)** connector on the mainboard. A pre-crimped
JST XH 4-pin cable (about 100 mm) is the easiest way to connect to it. Reading left to right,
the robot labels the pins **RX, 3.3V, TX, GND**. Cross the data lines:

| Robot pin | ESP32 pin |
|---|---|
| RX | TX |
| 3.3V | 3V3 |
| TX | RX |
| GND | GND |

* **Swapping TX and RX is the most common mistake.** If OpenNeato can't talk to the robot,
  check those two first. You can also change the UART pins in OpenNeato under
  *Settings → Device → UART Pins*.
* The robot's 3.3 V pin powers the ESP32, so it needs no USB cable, regulator or level
  shifter.
* **Flash OpenNeato over USB before you install the board.** Once it's inside the robot, USB
  is out of reach, and updates go over Wi-Fi (OTA). If the board can't join your Wi-Fi, it
  starts its own `neato-ap` access point.
* Wrap the board in foam, or insulate it some other way, so it can't short against the
  mainboard.
* Wi-Fi bursts draw roughly 300 mA. If the ESP32 resets at random, solder a 100–470 µF
  capacitor across its 3V3 and GND pins.

See OpenNeato's [user guide](https://github.com/renjfk/OpenNeato/blob/main/docs/user-guide.md)
for photos and the flashing tool.

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

## Floor plan

The **Plan** tab turns the map into a listing-style floor plan: walls drawn to their measured
thickness, doors with swings, each room's name and size (from the rooms you drew), and the
approximate interior area.

* **Rotation.** The plan (and the map view) is turned so the walls run straight across the
  screen. **⟲ 90° / ⟳ 90°** turns it a quarter turn. With **Tap a wall to → align to it**, tap
  any wall to make that wall horizontal. **Auto** goes back to the automatic choice.
* **Furniture.** Beds, sofas and cabinets against a wall are spotted and hidden. They show as
  dashed lines. With **Tap a wall to → hide / show it**, tap a dashed line to turn it into a wall,
  or tap a wall to hide it. Free-standing pieces such as a kitchen island are drawn as an
  outline.
* **Export.** Pick feet & inches or metres and Letter or A4 paper, give it a title, and press
  **Export SVG** or **Export PDF**. The plan is printed at the largest standard scale that fits
  (1:50, 1:100, …), with a scale bar.

Walls the robot never saw (behind a bed, say) are filled in by extending the walls it did see.
Measurements are only as good as the map: a map that's blurred or doubled gives a worse plan.
After mapping, the add-on re-aligns every scan against the whole map ("refining map"). This takes
a minute or more on small hardware. You can run it again from **Plan → Sharpen the map** after
extending a map.

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
