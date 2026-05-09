"""
SAR Drone — Hilly Terrain Mission Controller
=============================================
Modified mission for sar_hilly_terrain.sdf world.
Changes vs original:
  - Waypoints match hilly terrain survivor positions
  - Battery drains faster to trigger parachute demo
  - Precise timing from fault detection to parachute deploy
  - Detailed console output with timestamps
  - Low battery fault auto-injected at waypoint 5
"""

import time
import threading
import logging
import sys

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

from search_pattern import ExpandingSquareSearch
from vision_detector import SurvivorDetector
from safety_monitor import SafetyMonitor
from parachute_system import ParachuteDeploymentSystem
from mission_logger import MissionLogger

# ── Coloured console output ──────────────────────────────────────────────────
RED    = "\033[91m"; YELLOW = "\033[93m"; GREEN  = "\033[92m"
CYAN   = "\033[96m"; BOLD   = "\033[1m";  RESET  = "\033[0m"
BLUE   = "\033[94m"; MAG    = "\033[95m"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger("HillyMission")

# ── Configuration ─────────────────────────────────────────────────────────────
SITL_CONNECTION    = "tcp:127.0.0.1:5762"
TAKEOFF_ALTITUDE   = 20.0          # metres AGL
SEARCH_ALTITUDE    = 18.0          # search altitude (lower for hilly terrain)
CRUISE_SPEED       = 4.0           # m/s (slower for hills)
MAX_LEGS           = 8
SEARCH_SPACING     = 30.0          # metres

# SITL default home near Canberra — matches hilly terrain world origin
SEARCH_ORIGIN      = (-35.363261, 149.165230)

# Inject low-battery fault at this waypoint index (0-based)
FAULT_INJECT_AT_WP = 4

# Simulated battery drain — makes battery drop faster for demo
DEMO_BATT_DRAIN_START_PCT = 35   # battery % when fault injection begins
DEMO_BATT_DROP_TO_PCT     = 12   # battery % that triggers CRITICAL fault

# ── Timing tracker ────────────────────────────────────────────────────────────
class TimingTracker:
    def __init__(self):
        self.mission_start    = None
        self.fault_detected   = None
        self.parachute_deploy = None
        self.deploy_response  = None   # time from fault to deploy
        self.survivors_found  = []
        self.waypoints_visited = 0
        self.total_distance   = 0.0

    def mark_start(self):
        self.mission_start = time.time()
        print(f"\n{GREEN}{BOLD}{'═'*60}{RESET}")
        print(f"{GREEN}{BOLD}  MISSION START — {time.strftime('%H:%M:%S')}{RESET}")
        print(f"{GREEN}{BOLD}{'═'*60}{RESET}\n")

    def mark_fault(self, fault_code: str):
        self.fault_detected = time.time()
        elapsed = self.fault_detected - self.mission_start
        print(f"\n{RED}{BOLD}{'═'*60}{RESET}")
        print(f"{RED}{BOLD}  ⚠  FAULT DETECTED: {fault_code}{RESET}")
        print(f"{RED}{BOLD}  Mission time: {elapsed:.2f}s{RESET}")
        print(f"{RED}{BOLD}{'═'*60}{RESET}\n")

    def mark_deploy(self):
        self.parachute_deploy = time.time()
        if self.fault_detected:
            self.deploy_response = self.parachute_deploy - self.fault_detected
        elapsed = self.parachute_deploy - self.mission_start

        print(f"\n{YELLOW}{BOLD}{'═'*60}{RESET}")
        print(f"{YELLOW}{BOLD}  🪂  PARACHUTE DEPLOYED{RESET}")
        print(f"{YELLOW}{BOLD}{'═'*60}{RESET}")
        print(f"{YELLOW}  Total mission time    : {elapsed:.3f} s{RESET}")
        if self.deploy_response is not None:
            print(f"{RED}  Fault → Deploy time   : {self.deploy_response:.3f} s{RESET}")
            print(f"{CYAN}  Safety monitor rate   : 2 Hz (0.5s poll){RESET}")
            print(f"{CYAN}  Max theoretical delay : 0.5 s{RESET}")
            print(f"{CYAN}  Actual response time  : {self.deploy_response:.3f} s{RESET}")
        print(f"{YELLOW}  Waypoints visited      : {self.waypoints_visited} / {MAX_LEGS}{RESET}")
        print(f"{YELLOW}  Survivors found        : {len(self.survivors_found)}{RESET}")
        print(f"{YELLOW}  Distance covered       : {self.total_distance:.1f} m (est.){RESET}")
        print(f"{YELLOW}{'═'*60}{RESET}\n")

    def mark_survivor(self, lat, lon):
        ts = time.time() - self.mission_start
        self.survivors_found.append((lat, lon, ts))
        print(f"{CYAN}{BOLD}  ✓ SURVIVOR DETECTED @ T+{ts:.1f}s  ({lat:.6f}, {lon:.6f}){RESET}")

    def print_final(self):
        if not self.mission_start:
            return
        total = time.time() - self.mission_start
        print(f"\n{GREEN}{BOLD}{'═'*60}")
        print(f"  MISSION SUMMARY")
        print(f"{'═'*60}{RESET}")
        print(f"{GREEN}  Total flight time      : {total:.1f} s ({total/60:.1f} min){RESET}")
        print(f"{GREEN}  Waypoints completed    : {self.waypoints_visited} / {MAX_LEGS}{RESET}")
        print(f"{GREEN}  Survivors found        : {len(self.survivors_found)} / 3{RESET}")
        for i,(lat,lon,ts) in enumerate(self.survivors_found):
            print(f"{CYAN}    S-{i+1}: ({lat:.5f}, {lon:.5f}) @ T+{ts:.1f}s{RESET}")
        if self.deploy_response is not None:
            print(f"{RED}  Parachute response time: {self.deploy_response:.3f} s{RESET}")
            print(f"{RED}  Fault type             : CRITICAL_BATTERY{RESET}")
            print(f"{RED}  Deploy altitude        : ~{TAKEOFF_ALTITUDE:.0f} m AGL{RESET}")
        print(f"{GREEN}{'═'*60}{RESET}\n")


timing = TimingTracker()


# ── Mission class ─────────────────────────────────────────────────────────────
class HillyTerrainMission:
    def __init__(self):
        self.vehicle    = None
        self.detector   = SurvivorDetector()
        self.logger     = MissionLogger()
        self.parachute  = ParachuteDeploymentSystem()
        self._abort     = threading.Event()
        self._survivors = []

    # ── Connection ────────────────────────────────────────────────────────────
    def connect_vehicle(self):
        print(f"{BLUE}Connecting to SITL at {SITL_CONNECTION} …{RESET}")
        self.vehicle = connect(SITL_CONNECTION, wait_ready=True, timeout=60)
        print(f"{GREEN}Connected — {self.vehicle.version}{RESET}")
        print(f"{CYAN}Home: {self.vehicle.home_location}{RESET}")

        # Wire up safety monitor with timing callback
        self.safety = SafetyMonitorWithTiming(
            vehicle=self.vehicle,
            parachute=self.parachute,
            abort_event=self._abort,
            logger=self.logger,
            timing=timing,
        )
        self.parachute.attach_vehicle(self.vehicle)
        self.parachute.arm_system()

    # ── Arm and takeoff ───────────────────────────────────────────────────────
    def arm_and_takeoff(self, target_alt: float) -> bool:
        print(f"{BLUE}Pre-arm checks …{RESET}")
        while not self.vehicle.is_armable:
            print(f"  {YELLOW}Waiting for armable …{RESET}")
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(f"  {YELLOW}Waiting for arm …{RESET}")
            time.sleep(1)

        print(f"{GREEN}Armed — taking off to {target_alt:.0f}m …{RESET}")
        self.vehicle.simple_takeoff(target_alt)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            pct = min(100, alt/target_alt*100)
            bar = '█'*int(pct/5) + '░'*(20-int(pct/5))
            print(f"\r  Altitude: {alt:.1f}m [{bar}] {pct:.0f}%", end='', flush=True)
            if alt >= target_alt * 0.95:
                print(f"\n{GREEN}  Target altitude reached.{RESET}")
                return True
            if self._abort.is_set():
                return False
            time.sleep(0.5)

    # ── Navigation ────────────────────────────────────────────────────────────
    def goto(self, lat: float, lon: float, alt: float = SEARCH_ALTITUDE) -> bool:
        wp = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(wp, groundspeed=CRUISE_SPEED)
        while not self._abort.is_set():
            d = self._dist_to(lat, lon)
            if d < 3.0:
                return True
            time.sleep(0.5)
        return False

    def _dist_to(self, tlat, tlon) -> float:
        from math import radians, sin, cos, sqrt, atan2
        R = 6_371_000
        loc = self.vehicle.location.global_relative_frame
        φ1, φ2 = radians(loc.lat), radians(tlat)
        Δφ, Δλ = radians(tlat-loc.lat), radians(tlon-loc.lon)
        a = sin(Δφ/2)**2 + cos(φ1)*cos(φ2)*sin(Δλ/2)**2
        return R*2*atan2(sqrt(a), sqrt(1-a))

    # ── Battery fault injection ───────────────────────────────────────────────
    def inject_low_battery_fault(self):
        """Simulate battery dropping to critical level."""
        print(f"\n{RED}{BOLD}⚡ INJECTING LOW BATTERY FAULT …{RESET}")
        print(f"{RED}  Simulating rapid battery drain to {DEMO_BATT_DROP_TO_PCT}%{RESET}")
        # Write a trigger that safety monitor checks in its next poll
        import os
        try:
            with open("/tmp/sar_battery_fault.trigger", "w") as f:
                f.write(str(DEMO_BATT_DROP_TO_PCT))
        except Exception:
            pass
        # Directly call timing mark (safety monitor will deploy after next poll)
        timing.mark_fault("CRITICAL_BATTERY")

    # ── RTL ───────────────────────────────────────────────────────────────────
    def return_to_launch(self):
        print(f"{GREEN}Mission complete — RTL{RESET}")
        self.vehicle.mode = VehicleMode("RTL")

    # ── Main mission ──────────────────────────────────────────────────────────
    def run(self):
        try:
            self.connect_vehicle()
            self.safety.start()
            self.logger.start_mission()
            timing.mark_start()

            if not self.arm_and_takeoff(TAKEOFF_ALTITUDE):
                print(f"{RED}Takeoff aborted.{RESET}")
                return

            pattern = ExpandingSquareSearch(
                origin=SEARCH_ORIGIN,
                spacing=SEARCH_SPACING,
                legs=MAX_LEGS,
            )

            print(f"\n{CYAN}{'─'*60}")
            print(f"  STARTING HILLY TERRAIN SAR SEARCH")
            print(f"  Pattern: Expanding square  |  Legs: {MAX_LEGS}  |  Spacing: {SEARCH_SPACING}m")
            print(f"  Survivors: 3  |  Altitude: {SEARCH_ALTITUDE}m")
            print(f"{'─'*60}{RESET}\n")

            for i, (lat, lon) in enumerate(pattern.waypoints()):
                if self._abort.is_set():
                    print(f"{RED}Abort flag set — exiting search{RESET}")
                    break

                batt = self.vehicle.battery
                batt_pct = batt.level if batt and batt.level else 100
                batt_v   = f"{batt.voltage:.2f}V" if batt and batt.voltage else "?"

                print(f"{BLUE}WP {i:02d}/{MAX_LEGS-1} → ({lat:.5f}, {lon:.5f})"
                      f"  |  Batt: {batt_pct}% {batt_v}{RESET}")
                self.logger.log_waypoint(i, lat, lon)
                timing.waypoints_visited = i + 1

                # Inject low battery fault at specified waypoint
                if i == FAULT_INJECT_AT_WP and not timing.fault_detected:
                    self.inject_low_battery_fault()
                    # Give safety monitor time to detect and deploy
                    time.sleep(3)
                    if self._abort.is_set():
                        break

                arrived = self.goto(lat, lon)
                if not arrived:
                    break

                # Vision scan
                result = self.detector.scan_frame()
                if result:
                    print(f"{CYAN}  🔍 SURVIVOR DETECTED — conf={result.confidence:.2f}{RESET}")
                    self._survivors.append((lat, lon, time.time()))
                    timing.mark_survivor(lat, lon)
                    self.logger.log_survivor(lat, lon)

            if not self._abort.is_set():
                self.return_to_launch()

        except Exception as e:
            log.exception("Mission exception: %s", e)
        finally:
            timing.print_final()
            self.safety.stop()
            self.logger.end_mission(survivors=self._survivors)
            if self.vehicle:
                self.vehicle.close()


# ── Safety monitor with timing callback ──────────────────────────────────────
class SafetyMonitorWithTiming(SafetyMonitor):
    """Extends SafetyMonitor to record precise deployment timing."""
    def __init__(self, timing: TimingTracker, **kwargs):
        super().__init__(**kwargs)
        self._timing = timing

    def _handle_fault(self, fault: str):
        # Record timing before calling parent (which calls deploy)
        if not self._timing.fault_detected and fault in self.DEPLOY_IMMEDIATELY:
            self._timing.mark_fault(fault)
        super()._handle_fault(fault)
        if self._timing.fault_detected and not self._timing.parachute_deploy:
            if self.parachute.state.value == "DEPLOYED":
                self._timing.mark_deploy()

    @property
    def DEPLOY_IMMEDIATELY(self):
        from safety_monitor import FaultCode, DEPLOY_IMMEDIATELY as DI
        return DI


# ── Battery fault responder ───────────────────────────────────────────────────
def watch_battery_trigger(vehicle, safety_monitor, abort_event):
    """Background thread: watches for battery trigger file and forces fault."""
    import os
    trigger = "/tmp/sar_battery_fault.trigger"
    while not abort_event.is_set():
        if os.path.exists(trigger):
            try:
                target_pct = int(open(trigger).read().strip())
                os.remove(trigger)
                print(f"{RED}Battery trigger file detected — forcing CRITICAL_BATTERY fault{RESET}")
                # Force the safety monitor to fire by faking battery level
                # We do this by directly calling the fault handler
                safety_monitor._has_been_airborne = True
                from safety_monitor import FaultCode
                safety_monitor._handle_fault(FaultCode.CRITICAL_BATT
                    if hasattr(safety_monitor, 'CRITICAL_BATT')
                    else "CRITICAL_BATTERY")
            except Exception as e:
                print(f"{RED}Battery trigger error: {e}{RESET}")
        time.sleep(0.3)


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print(f"""
{CYAN}{BOLD}
  ╔══════════════════════════════════════════════════════╗
  ║   SAR DRONE — HILLY TERRAIN MISSION                 ║
  ║   World: sar_hilly_terrain.sdf                      ║
  ║   Survivors: 3  |  Trees: 15+  |  Boulders: 5      ║
  ║   Parachute fault: Low battery at WP {FAULT_INJECT_AT_WP:02d}            ║
  ╚══════════════════════════════════════════════════════╝
{RESET}
Terminal 1:  cd ardupilot && python Tools/autotest/sim_vehicle.py \\
             -v ArduCopter -f gazebo-iris --model JSON --console --map

Terminal 2:  export GZ_SIM_RESOURCE_PATH=$HOME/cognitive_drone/ardupilot_gazebo/models:\\
             $HOME/cognitive_drone/worlds
             gz sim -v4 -r sar_hilly_terrain.sdf

Terminal 3:  python gazebo_parachute_bridge_hilly.py

Then run:    python mission_controller_hilly.py
""")

    mission = HillyTerrainMission()
    mission.run()
