# Autonomous SAR Drone — Smart Parachute Deployment System

A digital-twin simulation of an Autonomous Search and Rescue drone built on
**ArduPilot SITL** + **Gazebo Harmonic**, with an **OpenCV** survivor detector
and a real-time **parachute safety system**.

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│ SIMULATION LAYER                                        │
│  ArduPilot SITL ──MAVLink──► Gazebo Harmonic            │
└───────────────────────────────┬─────────────────────────┘
                                │ DroneKit / MAVSDK
┌───────────────────────────────▼─────────────────────────┐
│ mission_controller.py                                   │
│  ├── search_pattern.py    Expanding-square GPS path     │
│  ├── vision_detector.py   OpenCV HSV + contour detect   │
│  ├── safety_monitor.py    Telemetry watchdog thread     │
│  ├── parachute_system.py  Servo deploy + motor kill     │
│  └── mission_logger.py    CSV + JSON event log          │
└─────────────────────────────────────────────────────────┘
```

---

## Prerequisites

| Tool | Version | Notes |
|------|---------|-------|
| Python | 3.10+ | |
| ArduPilot | 4.5+ | Build from source or use pre-built SITL |
| Gazebo Harmonic | 8.x | `gz sim` |
| MAVLink / DroneKit | see requirements.txt | |
| OpenCV | 4.9 | `pip install opencv-python` |

---

## Installation

```bash
# 1. Clone or copy this project
git clone <your-repo> sar_drone && cd sar_drone

# 2. Create a virtual environment
python -m venv .venv && source .venv/bin/activate

# 3. Install Python dependencies
pip install -r requirements.txt

# 4. Install ArduPilot SITL (Ubuntu)
# Follow: https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
#   cd ArduPilot
#   Tools/environment_install/install-prereqs-ubuntu.sh -y
#   . ~/.profile
#   ./waf configure --board sitl
#   ./waf copter
```

---

## Running the Simulation

### Step 1 — Start ArduPilot SITL

```bash
# In ArduPilot repo root
sim_vehicle.py -v ArduCopter --console --map
# Default TCP port: 5762 (matches SITL_CONNECTION in mission_controller.py)
```

### Step 2 — (Optional) Launch Gazebo Harmonic

```bash
gz sim -v4 -r worlds/quadcopter.sdf
# Or use the ArduPilot Gazebo plugin:
# https://github.com/ArduPilot/ardupilot_gazebo
```

### Step 3 — Run the SAR Mission

```bash
python mission_controller.py
```

---

## Module Reference

### `mission_controller.py`
Entry point. Connects to SITL, arms the drone, executes the expanding-square
search, and tears everything down on completion or abort.

Key constants:
- `TAKEOFF_ALTITUDE` — altitude before beginning the search (default 20 m)
- `SEARCH_ALTITUDE` — altitude during the sweep (default 15 m)
- `MAX_LEGS` — how many legs in the expanding square (default 8)

---

### `search_pattern.py`
Generates GPS waypoints for an expanding-square SAR pattern.

```python
from search_pattern import ExpandingSquareSearch

pattern = ExpandingSquareSearch(
    origin=(-35.363261, 149.165230),
    spacing=30,   # metres between legs
    legs=8,
)
for lat, lon in pattern.waypoints():
    print(lat, lon)
```

Run standalone to visualise the ASCII map:
```bash
python search_pattern.py
```

---

### `vision_detector.py`
OpenCV pipeline: HSV masking → morphological clean-up → contour detection →
aspect-ratio filter → confidence scoring.

```python
from vision_detector import SurvivorDetector

detector = SurvivorDetector(use_camera=False, show_window=True)
result = detector.scan_frame()   # returns Detection or None
```

Set `use_camera=True` to read from a real camera / Gazebo video stream.
Tune `HSV_LOWER_1 / HSV_UPPER_1` for your target colour.

Run standalone for a 30-frame test:
```bash
python vision_detector.py
```

---

### `safety_monitor.py`
Background thread monitoring:
- Battery voltage & remaining %
- GPS fix type & HDOP
- EKF health
- Attitude (roll/pitch limits)
- Altitude free-fall guard
- MAVLink heartbeat timeout

On critical fault → calls `parachute.deploy()` and sets `abort_event`.
On soft fault (low battery, GPS degraded) → sets `abort_event` only.

---

### `parachute_system.py`
Sends `MAV_CMD_DO_SET_SERVO` to two channels:
1. Parachute release servo (ch 9, 2000 PWM)
2. Motor kill switch (ch 10, 2000 PWM)
Then calls `vehicle.armed = False`.

Without a vehicle attached, logs a simulation stub so you can test offline.

States: `LOCKED → ARMED → DEPLOYED / FAILED`

---

### `mission_logger.py`
Writes two files to `./logs/`:
- `sar_mission_<ts>.json` — structured summary (survivors, faults, path)
- `sar_events_<ts>.csv`   — timestamped raw event stream

---

## Customisation Tips

| Goal | Where to change |
|------|-----------------|
| Different search origin | `SEARCH_ORIGIN` in `mission_controller.py` |
| More search area | Increase `MAX_LEGS` or `SEARCH_SPACING` |
| Different target colour | `HSV_LOWER_1/2` in `vision_detector.py` |
| Stricter safety limits | Threshold constants in `safety_monitor.py` |
| Real parachute wiring | `PARACHUTE_SERVO_CHANNEL` in `parachute_system.py` |
| RTK GPS (lower HDOP) | `GPS_HDOP_MAX` in `safety_monitor.py` |

---

## Gazebo Integration Notes

To connect ArduPilot SITL with Gazebo Harmonic:
1. Install `ardupilot_gazebo` plugin
2. Launch with `gazebo_ardupilot_sitl.sh` (provided in that plugin repo)
3. The drone model includes a downward-facing camera; pipe its video to
   `vision_detector.py` via `CAMERA_SOURCE = "rtsp://..."` or OpenCV GStreamer.

For the parachute visual in Gazebo, add a prismatic joint on the canopy model
and trigger it via `gz topic` when the servo channel fires.

---

## License

MIT — free to use, modify, and distribute for research and education.
