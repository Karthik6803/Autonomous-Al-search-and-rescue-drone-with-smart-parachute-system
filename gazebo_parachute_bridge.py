"""
Gazebo Parachute Deployment Bridge
====================================
Watches for /tmp/sar_parachute_deploy.trigger, then animates
the parachute model rising above the drone and descending slowly.
"""

import subprocess
import threading
import time
import os
import logging

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s")
log = logging.getLogger("ParachuteBridge")

TRIGGER_FILE     = "/tmp/sar_parachute_deploy.trigger"
POLL_INTERVAL    = 0.2
WORLD_NAME       = "sar_parachute_demo"   # must match <world name="..."> in SDF
CHUTE_MODEL      = "parachute"
DRONE_MODEL      = "iris_with_ardupilot"

DEPLOY_HEIGHT    = 18.0   # metres — parachute pops up here (above drone)
LAND_HEIGHT      = 0.5    # metres — where it settles
DESCENT_DURATION = 10.0   # seconds for full descent
DESCENT_STEPS    = 60     # smoothness

RED    = "\033[91m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


def banner(msg, colour=RED):
    w = 62
    print(f"\n{colour}{BOLD}{'═'*w}{RESET}")
    print(f"{colour}{BOLD}  {msg}{RESET}")
    print(f"{colour}{BOLD}{'═'*w}{RESET}\n")


def gz_set_pose(model: str, x: float, y: float, z: float):
    """Move a model to (x,y,z) using gz service set_pose."""
    req = (
        f'name: "{model}" '
        f'position {{ x: {x:.3f} y: {y:.3f} z: {z:.3f} }} '
        f'orientation {{ x: 0 y: 0 z: 0 w: 1 }}'
    )
    cmd = [
        "gz", "service",
        "-s", f"/world/{WORLD_NAME}/set_pose",
        "--reqtype",  "gz.msgs.Pose",
        "--reptype",  "gz.msgs.Boolean",
        "--timeout",  "1000",
        "--req", req
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
        if result.returncode != 0:
            log.debug("set_pose stderr: %s", result.stderr.strip())
    except Exception as e:
        log.debug("gz_set_pose error (non-fatal): %s", e)


def get_drone_xy() -> tuple:
    """Read drone pose from Gazebo topic. Returns (x, y) or (0, 0)."""
    try:
        result = subprocess.run(
            ["gz", "topic", "-e", "-n", "1",
             "-t", f"/world/{WORLD_NAME}/dynamic_pose/info"],
            capture_output=True, text=True, timeout=3
        )
        # Look for iris_with_ardupilot block and parse x/y
        lines = result.stdout.splitlines()
        in_drone = False
        x, y = 0.0, 0.0
        for line in lines:
            if DRONE_MODEL in line:
                in_drone = True
            if in_drone and "x:" in line and "y:" not in line:
                try:
                    x = float(line.split(":")[1].strip())
                except Exception:
                    pass
            if in_drone and "y:" in line and "x:" not in line:
                try:
                    y = float(line.split(":")[1].strip())
                    break
                except Exception:
                    pass
        return x, y
    except Exception:
        return 0.0, 0.0


def animate_deployment(reason: str):
    banner(f"⚠  FAULT: {reason}", RED)
    time.sleep(0.2)
    banner("🪂  PARACHUTE DEPLOYING!", YELLOW)

    # Get drone XY position
    dx, dy = get_drone_xy()
    log.info("Drone XY: (%.2f, %.2f) — deploying parachute above", dx, dy)

    # Step 1: instantly pop parachute above drone
    gz_set_pose(CHUTE_MODEL, dx, dy, DEPLOY_HEIGHT)
    log.info("Parachute placed at z=%.1f m", DEPLOY_HEIGHT)
    time.sleep(0.5)

    # Step 2: animate slow descent
    log.info("Descending from %.1f m to %.1f m over %.0f s …",
             DEPLOY_HEIGHT, LAND_HEIGHT, DESCENT_DURATION)

    for step in range(DESCENT_STEPS + 1):
        frac  = step / DESCENT_STEPS
        # Ease-in-out for natural feel
        ease  = frac * frac * (3 - 2 * frac)
        cur_z = DEPLOY_HEIGHT + (LAND_HEIGHT - DEPLOY_HEIGHT) * ease
        # Slight horizontal drift (wind effect)
        cur_x = dx + frac * 1.5
        cur_y = dy + frac * 0.8
        gz_set_pose(CHUTE_MODEL, cur_x, cur_y, cur_z)
        time.sleep(DESCENT_DURATION / DESCENT_STEPS)

    banner("✅  DRONE LANDED SAFELY VIA PARACHUTE", GREEN)
    log.info("Descent complete. Final position: (%.2f, %.2f, %.2f)",
             dx + 1.5, dy + 0.8, LAND_HEIGHT)


def watch_for_trigger():
    # Clean up stale trigger
    if os.path.exists(TRIGGER_FILE):
        os.remove(TRIGGER_FILE)

    log.info("Watching for trigger at %s …", TRIGGER_FILE)
    log.info("Start mission: python mission_controller.py")
    log.info("Force trigger: python trigger_parachute_demo.py attitude")

    deployed = False
    while True:
        if not deployed and os.path.exists(TRIGGER_FILE):
            deployed = True
            reason = open(TRIGGER_FILE).read().strip()
            log.critical("TRIGGER DETECTED — %s", reason)
            threading.Thread(
                target=animate_deployment, args=(reason,), daemon=True
            ).start()
        time.sleep(POLL_INTERVAL)


if __name__ == "__main__":
    print(f"""
{CYAN}{BOLD}
  ╔════════════════════════════════════════════════════╗
  ║   SAR DRONE — Gazebo Parachute Bridge             ║
  ║   World  : {WORLD_NAME:<40}║
  ║   Model  : {CHUTE_MODEL:<40}║
  ║   Deploy : {DEPLOY_HEIGHT}m → {LAND_HEIGHT}m over {DESCENT_DURATION}s{' '*22}║
  ╚════════════════════════════════════════════════════╝
{RESET}""")
    watch_for_trigger()
