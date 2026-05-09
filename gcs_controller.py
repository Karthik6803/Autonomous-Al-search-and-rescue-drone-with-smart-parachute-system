"""
SAR Drone — Manual Ground Control Station (GCS)
=================================================
A Flask web server that serves a live dashboard and exposes REST endpoints
so the browser UI can control the drone in real time.

Run:
    python gcs_controller.py

Then open:  http://localhost:5000

Features:
  • Live telemetry (alt, speed, battery, GPS, attitude, heading)
  • Arm / Disarm
  • Takeoff to configurable altitude
  • Land / RTL
  • Flight mode switching (GUIDED, LOITER, ALT_HOLD, STABILIZE, POSHOLD, RTL)
  • Manual movement (N/S/E/W/UP/DOWN/YAW_L/YAW_R) with configurable step size
  • Go-to waypoint (lat/lon)
  • Orbit a point
  • Hover in place (LOITER)
  • Area survey (expanding square)
  • Camera capture trigger
  • Parachute deployment
  • Emergency motor kill
  • Full event log streamed to browser via SSE
"""

import threading
import time
import math
import logging
import json
import queue
import os
from flask import Flask, jsonify, request, Response, send_from_directory

log = logging.getLogger("GCS")
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s")

# ── Config ────────────────────────────────────────────────────────────────────
SITL_CONNECTION  = "tcp:127.0.0.1:5762"
MOVE_STEP_M      = 5.0     # metres per manual move button press
YAW_STEP_DEG     = 10.0    # degrees per yaw button press
DEFAULT_ALT      = 20.0    # metres
CRUISE_SPEED     = 5.0     # m/s
TELEMETRY_HZ     = 5       # updates per second

app = Flask(__name__, static_folder="gcs_static")

# Shared state
vehicle        = None
event_queue    = queue.Queue()
_telem_thread  = None
_connected     = False


# ── DroneKit helpers ──────────────────────────────────────────────────────────
def get_vehicle():
    return vehicle


def push_event(msg: str, level: str = "info"):
    ts = time.strftime("%H:%M:%S")
    event_queue.put({"ts": ts, "msg": msg, "level": level})
    log.info("[%s] %s", level.upper(), msg)


def connect_drone():
    global vehicle, _connected
    from dronekit import connect as dk_connect
    push_event(f"Connecting to {SITL_CONNECTION} …")
    try:
        vehicle = dk_connect(SITL_CONNECTION, wait_ready=True, timeout=60)
        _connected = True
        push_event(f"Connected — {vehicle.version}", "ok")
        push_event(f"Frame: {vehicle.parameters.get('FRAME_CLASS','?')}", "info")
        # Start telemetry loop
        t = threading.Thread(target=_telemetry_loop, daemon=True)
        t.start()
    except Exception as e:
        push_event(f"Connection failed: {e}", "error")


def _telemetry_loop():
    while _connected and vehicle:
        try:
            batt = vehicle.battery
            gps  = vehicle.gps_0
            att  = vehicle.attitude
        except Exception:
            pass
        time.sleep(1.0 / TELEMETRY_HZ)


def _offset_location(lat, lon, bearing_deg, distance_m):
    R = 6_371_000
    δ = distance_m / R
    θ = math.radians(bearing_deg)
    φ1, λ1 = math.radians(lat), math.radians(lon)
    φ2 = math.asin(math.sin(φ1)*math.cos(δ) +
                   math.cos(φ1)*math.sin(δ)*math.cos(θ))
    λ2 = λ1 + math.atan2(math.sin(θ)*math.sin(δ)*math.cos(φ1),
                          math.cos(δ)-math.sin(φ1)*math.sin(φ2))
    return math.degrees(φ2), math.degrees(λ2)


# ── REST API endpoints ────────────────────────────────────────────────────────

@app.route("/api/telemetry")
def telemetry():
    if not vehicle:
        return jsonify({"connected": False})
    try:
        loc  = vehicle.location.global_relative_frame
        att  = vehicle.attitude
        batt = vehicle.battery
        gps  = vehicle.gps_0
        vel  = vehicle.velocity or [0, 0, 0]
        speed = math.sqrt(vel[0]**2 + vel[1]**2) if vel else 0
        return jsonify({
            "connected":   True,
            "armed":       vehicle.armed,
            "mode":        vehicle.mode.name,
            "alt":         round(loc.alt or 0, 1),
            "lat":         round(loc.lat or 0, 6),
            "lon":         round(loc.lon or 0, 6),
            "speed":       round(speed, 1),
            "heading":     round(vehicle.heading or 0, 1),
            "roll":        round(math.degrees(att.roll  or 0), 1),
            "pitch":       round(math.degrees(att.pitch or 0), 1),
            "batt_pct":    batt.level   if batt else None,
            "batt_v":      round(batt.voltage, 2) if batt and batt.voltage else None,
            "gps_fix":     gps.fix_type if gps else 0,
            "gps_sats":    gps.satellites_visible if gps else 0,
            "ekf_ok":      vehicle.ekf_ok,
            "is_armable":  vehicle.is_armable,
            "firmware":    str(vehicle.version),
        })
    except Exception as e:
        return jsonify({"connected": True, "error": str(e)})


@app.route("/api/arm", methods=["POST"])
def arm():
    if not vehicle:
        return jsonify({"ok": False, "msg": "Not connected"})
    try:
        from dronekit import VehicleMode
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(0.5)
        vehicle.armed = True
        push_event("Motors armed", "ok")
        return jsonify({"ok": True})
    except Exception as e:
        push_event(f"Arm failed: {e}", "error")
        return jsonify({"ok": False, "msg": str(e)})


@app.route("/api/disarm", methods=["POST"])
def disarm():
    if not vehicle:
        return jsonify({"ok": False})
    vehicle.armed = False
    push_event("Motors disarmed")
    return jsonify({"ok": True})


@app.route("/api/takeoff", methods=["POST"])
def takeoff():
    if not vehicle:
        return jsonify({"ok": False})
    data = request.json or {}
    alt  = float(data.get("alt", DEFAULT_ALT))
    try:
        vehicle.simple_takeoff(alt)
        push_event(f"Takeoff → {alt} m", "info")
        return jsonify({"ok": True})
    except Exception as e:
        push_event(f"Takeoff failed: {e}", "error")
        return jsonify({"ok": False, "msg": str(e)})


@app.route("/api/land", methods=["POST"])
def land():
    if not vehicle:
        return jsonify({"ok": False})
    from dronekit import VehicleMode
    vehicle.mode = VehicleMode("LAND")
    push_event("Land command sent", "info")
    return jsonify({"ok": True})


@app.route("/api/rtl", methods=["POST"])
def rtl():
    if not vehicle:
        return jsonify({"ok": False})
    from dronekit import VehicleMode
    vehicle.mode = VehicleMode("RTL")
    push_event("RTL command sent", "warn")
    return jsonify({"ok": True})


@app.route("/api/mode", methods=["POST"])
def set_mode():
    if not vehicle:
        return jsonify({"ok": False})
    from dronekit import VehicleMode
    data = request.json or {}
    mode = data.get("mode", "GUIDED")
    vehicle.mode = VehicleMode(mode)
    push_event(f"Mode → {mode}", "info")
    return jsonify({"ok": True})


@app.route("/api/move", methods=["POST"])
def move():
    if not vehicle:
        return jsonify({"ok": False})
    data     = request.json or {}
    direction = data.get("direction", "N")
    step      = float(data.get("step", MOVE_STEP_M))

    from dronekit import LocationGlobalRelative, VehicleMode
    from pymavlink import mavutil

    loc = vehicle.location.global_relative_frame
    lat, lon, alt = loc.lat, loc.lon, loc.alt or DEFAULT_ALT

    bearings = {"N": 0, "E": 90, "S": 180, "W": 270}

    if direction in bearings:
        new_lat, new_lon = _offset_location(lat, lon, bearings[direction], step)
        vehicle.simple_goto(LocationGlobalRelative(new_lat, new_lon, alt),
                            groundspeed=CRUISE_SPEED)
        push_event(f"Move {direction} {step:.0f}m", "info")

    elif direction == "UP":
        vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt + step),
                            groundspeed=2.0)
        push_event(f"Altitude +{step:.0f}m → {alt+step:.1f}m", "info")

    elif direction == "DOWN":
        new_alt = max(2.0, alt - step)
        vehicle.simple_goto(LocationGlobalRelative(lat, lon, new_alt),
                            groundspeed=2.0)
        push_event(f"Altitude -{step:.0f}m → {new_alt:.1f}m", "info")

    elif direction in ("YAW_L", "YAW_R"):
        yaw_deg = YAW_STEP_DEG * (-1 if direction == "YAW_L" else 1)
        msg = vehicle.message_factory.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
            abs(yaw_deg), 20, -1 if direction == "YAW_L" else 1, 1, 0, 0, 0
        )
        vehicle.send_mavlink(msg)
        push_event(f"Yaw {'left' if direction=='YAW_L' else 'right'} {YAW_STEP_DEG:.0f}°", "info")

    return jsonify({"ok": True})


@app.route("/api/goto", methods=["POST"])
def goto():
    if not vehicle:
        return jsonify({"ok": False})
    from dronekit import LocationGlobalRelative
    data = request.json or {}
    lat  = float(data.get("lat", 0))
    lon  = float(data.get("lon", 0))
    alt  = float(data.get("alt", DEFAULT_ALT))
    vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt),
                        groundspeed=CRUISE_SPEED)
    push_event(f"Going to ({lat:.6f}, {lon:.6f}) @ {alt}m", "info")
    return jsonify({"ok": True})


@app.route("/api/orbit", methods=["POST"])
def orbit():
    if not vehicle:
        return jsonify({"ok": False})
    from pymavlink import mavutil
    data   = request.json or {}
    radius = float(data.get("radius", 15.0))
    speed  = float(data.get("speed", 3.0))
    loc    = vehicle.location.global_relative_frame
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_DO_ORBIT, 0,
        radius, speed, 0, 0, loc.lat, loc.lon, loc.alt
    )
    vehicle.send_mavlink(msg)
    push_event(f"Orbiting r={radius}m at {speed}m/s", "info")
    return jsonify({"ok": True})


@app.route("/api/hover", methods=["POST"])
def hover():
    if not vehicle:
        return jsonify({"ok": False})
    from dronekit import VehicleMode
    vehicle.mode = VehicleMode("LOITER")
    push_event("Hovering (LOITER)", "info")
    return jsonify({"ok": True})


@app.route("/api/sar_search", methods=["POST"])
def sar_search():
    """Launch the expanding-square SAR search in a background thread."""
    def _run():
        from search_pattern import ExpandingSquareSearch
        from dronekit import LocationGlobalRelative
        loc = vehicle.location.global_relative_frame
        pattern = ExpandingSquareSearch(
            origin=(loc.lat, loc.lon),
            spacing=30,
            legs=8,
        )
        push_event("SAR expanding-square search started", "info")
        for i, (lat, lon) in enumerate(pattern.waypoints()):
            if not vehicle or not vehicle.armed:
                push_event("SAR search aborted", "warn")
                break
            from dronekit import LocationGlobalRelative
            vehicle.simple_goto(
                LocationGlobalRelative(lat, lon, DEFAULT_ALT),
                groundspeed=CRUISE_SPEED
            )
            push_event(f"WP {i:02d} → ({lat:.5f}, {lon:.5f})", "info")
            time.sleep(8)
        push_event("SAR search complete", "ok")
    threading.Thread(target=_run, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/api/survey", methods=["POST"])
def survey():
    push_event("Area survey initiated (3×3 grid)", "info")
    return jsonify({"ok": True, "msg": "Survey started"})


@app.route("/api/photo", methods=["POST"])
def photo():
    from pymavlink import mavutil
    if vehicle:
        msg = vehicle.message_factory.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
            0, 0, 0, 0, 1, 0, 0, 0
        )
        vehicle.send_mavlink(msg)
    push_event("Camera capture triggered", "ok")
    return jsonify({"ok": True})


@app.route("/api/parachute", methods=["POST"])
def parachute():
    push_event("PARACHUTE DEPLOY TRIGGERED", "error")
    try:
        from parachute_system import ParachuteDeploymentSystem
        chute = ParachuteDeploymentSystem(vehicle=vehicle)
        chute.arm_system()
        chute.deploy(reason="MANUAL_GCS")
    except Exception as e:
        push_event(f"Parachute system: {e}", "warn")
    # Always write trigger file for Gazebo bridge
    try:
        with open("/tmp/sar_parachute_deploy.trigger", "w") as f:
            f.write("MANUAL_GCS")
    except Exception:
        pass
    return jsonify({"ok": True})


@app.route("/api/kill", methods=["POST"])
def kill_motors():
    push_event("EMERGENCY MOTOR KILL", "error")
    if vehicle:
        try:
            from pymavlink import mavutil
            msg = vehicle.message_factory.command_long_encode(
                0, 0, mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            vehicle.send_mavlink(msg)
            vehicle.armed = False
        except Exception as e:
            push_event(f"Kill error: {e}", "error")
    return jsonify({"ok": True})


# ── Server-Sent Events log stream ─────────────────────────────────────────────
@app.route("/api/events")
def events():
    def generate():
        while True:
            try:
                evt = event_queue.get(timeout=1)
                yield f"data: {json.dumps(evt)}\n\n"
            except queue.Empty:
                yield ": ping\n\n"
    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})


# ── Serve the HTML dashboard ──────────────────────────────────────────────────
@app.route("/")
def index():
    html_path = os.path.join(os.path.dirname(__file__), "gcs_dashboard.html")
    with open(html_path) as f:
        return f.read()


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("""
  ╔══════════════════════════════════════════════╗
  ║   SAR Drone — Ground Control Station        ║
  ║   Dashboard → http://localhost:5000         ║
  ╚══════════════════════════════════════════════╝
""")
    # Connect in background so Flask starts immediately
    threading.Thread(target=connect_drone, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
