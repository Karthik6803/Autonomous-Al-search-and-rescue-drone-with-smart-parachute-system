"""
SAR Drone Mission Controller
Entry point: orchestrates search, vision, and safety subsystems.
"""

import time
import threading
import logging
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

from search_pattern import ExpandingSquareSearch
from vision_detector import SurvivorDetector
from safety_monitor import SafetyMonitor
from parachute_system import ParachuteDeploymentSystem
from mission_logger import MissionLogger

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s"
)
log = logging.getLogger("MissionController")

# ──────────────────────────────────────────────
# Configuration
# ──────────────────────────────────────────────
SITL_CONNECTION = "tcp:127.0.0.1:5762"  # ArduPilot SITL default
TAKEOFF_ALTITUDE = 20.0          # metres
SEARCH_ALTITUDE  = 15.0          # metres AGL during sweep
CRUISE_SPEED     = 5.0           # m/s
SEARCH_ORIGIN    = (-35.363261, 149.165230)  # lat, lon (SITL default home)
SEARCH_SPACING   = 30.0          # metres between sweep legs
MAX_LEGS         = 8             # number of expanding-square legs


class SARMission:
    def __init__(self):
        self.vehicle      = None
        self.detector     = SurvivorDetector()
        self.logger       = MissionLogger()
        self.parachute    = ParachuteDeploymentSystem()
        self.safety       = None          # initialised after vehicle connect
        self._abort       = threading.Event()
        self._survivors   = []

    # ── Connection & arming ──────────────────────────────────────────────────
    def connect_vehicle(self):
        log.info("Connecting to SITL at %s …", SITL_CONNECTION)
        self.vehicle = connect(SITL_CONNECTION, wait_ready=True, timeout=60)
        log.info("Connected. Firmware: %s", self.vehicle.version)

        self.safety = SafetyMonitor(
            vehicle=self.vehicle,
            parachute=self.parachute,
            abort_event=self._abort,
            logger=self.logger,
        )
        self.parachute.attach_vehicle(self.vehicle)
        self.parachute.arm_system()

    def arm_and_takeoff(self, target_alt: float):
        log.info("Pre-arm checks …")
        while not self.vehicle.is_armable:
            log.info("  Waiting for vehicle to become armable …")
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            log.info("  Waiting for arm confirmation …")
            time.sleep(1)

        log.info("Taking off to %.1f m …", target_alt)
        self.vehicle.simple_takeoff(target_alt)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            log.info("  Altitude: %.1f m", alt)
            if alt >= target_alt * 0.95:
                log.info("Target altitude reached.")
                break
            if self._abort.is_set():
                return False
            time.sleep(1)
        return True

    # ── Waypoint navigation ──────────────────────────────────────────────────
    def goto(self, lat: float, lon: float, alt: float = SEARCH_ALTITUDE):
        """Send a single GOTO command and wait for arrival within 3 m."""
        wp = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(wp, groundspeed=CRUISE_SPEED)

        while not self._abort.is_set():
            rem = self._distance_to(lat, lon)
            if rem < 3.0:
                return True
            time.sleep(0.5)
        return False

    def _distance_to(self, target_lat, target_lon) -> float:
        from math import radians, sin, cos, sqrt, atan2
        R = 6_371_000
        loc = self.vehicle.location.global_relative_frame
        φ1, φ2 = radians(loc.lat), radians(target_lat)
        Δφ = radians(target_lat - loc.lat)
        Δλ = radians(target_lon - loc.lon)
        a = sin(Δφ/2)**2 + cos(φ1)*cos(φ2)*sin(Δλ/2)**2
        return R * 2 * atan2(sqrt(a), sqrt(1 - a))

    # ── RTL ──────────────────────────────────────────────────────────────────
    def return_to_launch(self):
        log.info("Initiating RTL …")
        self.vehicle.mode = VehicleMode("RTL")

    # ── Main mission loop ────────────────────────────────────────────────────
    def run(self):
        try:
            self.connect_vehicle()
            self.safety.start()
            self.logger.start_mission()

            if not self.arm_and_takeoff(TAKEOFF_ALTITUDE):
                log.warning("Takeoff aborted.")
                return

            pattern = ExpandingSquareSearch(
                origin=SEARCH_ORIGIN,
                spacing=SEARCH_SPACING,
                legs=MAX_LEGS,
            )

            log.info("Starting expanding-square search — %d legs", MAX_LEGS)
            for wp_index, (lat, lon) in enumerate(pattern.waypoints()):
                if self._abort.is_set():
                    log.warning("Abort flag set — exiting search loop")
                    break

                log.info("WP %02d → (%.6f, %.6f)", wp_index, lat, lon)
                self.logger.log_waypoint(wp_index, lat, lon)

                arrived = self.goto(lat, lon)
                if not arrived:
                    break

                # ── Vision scan at each waypoint ──────────────────────────
                result = self.detector.scan_frame()
                if result:
                    log.info("🟥 SURVIVOR DETECTED at (%.6f, %.6f)", lat, lon)
                    self._survivors.append((lat, lon, time.time()))
                    self.logger.log_survivor(lat, lon)

            if not self._abort.is_set():
                log.info("Search pattern complete.")
                self.return_to_launch()
            else:
                log.warning("Mission aborted. Parachute may have deployed.")

        except Exception as exc:
            log.exception("Unhandled mission exception: %s", exc)
        finally:
            self.safety.stop()
            self.logger.end_mission(survivors=self._survivors)
            if self.vehicle:
                self.vehicle.close()
            log.info("Mission complete. Survivors found: %d", len(self._survivors))


if __name__ == "__main__":
    mission = SARMission()
    mission.run()