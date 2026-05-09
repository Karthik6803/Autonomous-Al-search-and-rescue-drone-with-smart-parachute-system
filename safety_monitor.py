"""
Safety Monitor
Runs in a background thread, polling vehicle telemetry every 0.5 s.
Triggers parachute deployment and mission abort on threshold violations.

Monitored channels:
  • Battery voltage        < BATT_VOLTAGE_MIN  V
  • Battery remaining      < BATT_REMAINING_MIN %
  • GPS fix type           < GPS_FIX_MIN  (3 = 3D fix required)
  • GPS HDOP               > GPS_HDOP_MAX
  • EKF status flags       any error bit set
  • Altitude               < ALT_FLOOR_M  (structural free-fall indicator)
  • Attitude (roll/pitch)  > ATTITUDE_MAX_DEG  (flip / spin indicator)
  • Heartbeat timeout      > HEARTBEAT_TIMEOUT_S
"""

import threading
import time
import logging
from dronekit import Vehicle

log = logging.getLogger("SafetyMonitor")


# ──────────────────────────────────────────────
# Thresholds — tune for your airframe
# ──────────────────────────────────────────────
BATT_VOLTAGE_MIN    = 10.5    # V   (3S LiPo critical)
BATT_REMAINING_MIN  = 15      # %
GPS_FIX_MIN         = 3       # 3 = 3D fix
GPS_HDOP_MAX        = 2.5     # dimensionless
ALT_FLOOR_M         = 2.0     # below this = near-crash → deploy
ATTITUDE_MAX_DEG    = 60.0    # roll or pitch beyond this = loss of control
HEARTBEAT_TIMEOUT_S = 5.0     # seconds without MAVLink heartbeat


class FaultCode:
    NONE            = "NONE"
    LOW_BATTERY     = "LOW_BATTERY"
    CRITICAL_BATT   = "CRITICAL_BATTERY"
    GPS_LOSS        = "GPS_LOSS"
    EKF_ERROR       = "EKF_ERROR"
    ATTITUDE_FLIP   = "ATTITUDE_FLIP"
    FREEFALL        = "FREEFALL"
    HEARTBEAT_LOSS  = "HEARTBEAT_LOSS"


# Faults that immediately deploy the parachute (no grace period)
DEPLOY_IMMEDIATELY = {
    FaultCode.CRITICAL_BATT,
    FaultCode.FREEFALL,
    FaultCode.ATTITUDE_FLIP,
    FaultCode.HEARTBEAT_LOSS,
    FaultCode.EKF_ERROR,
}


class SafetyMonitor:
    def __init__(self, vehicle: Vehicle, parachute, abort_event: threading.Event, logger):
        self.vehicle       = vehicle
        self.parachute     = parachute
        self.abort_event   = abort_event
        self.logger        = logger

        self._thread       = threading.Thread(target=self._run, daemon=True, name="SafetyMonitor")
        self._stop_event   = threading.Event()
        self._last_heartbeat = time.time()
        self._fault_active = FaultCode.NONE
        self._has_been_airborne = False
        self._start_time = time.time()
        self._grace_period = 10.0   # seconds — ignore faults during startup

        # Register heartbeat listener
        self.vehicle.add_message_listener("HEARTBEAT", self._on_heartbeat)

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def start(self):
        log.info("Safety monitor starting …")
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=3)
        log.info("Safety monitor stopped.")

    # ── Heartbeat listener ────────────────────────────────────────────────────
    def _on_heartbeat(self, vehicle, name, message):
        self._last_heartbeat = time.time()

    # ── Main polling loop ─────────────────────────────────────────────────────
    def _run(self):
        while not self._stop_event.is_set():
            try:
                fault = self._assess()
                if fault != FaultCode.NONE:
                    self._handle_fault(fault)
            except Exception as exc:
                log.exception("Safety monitor error: %s", exc)
            time.sleep(0.5)

    def _assess(self) -> str:
        # Ignore all faults during the startup grace period
        if time.time() - self._start_time < self._grace_period:
            return FaultCode.NONE

        # ── 1. Heartbeat ────────────────────────────────────────────────────
        if time.time() - self._last_heartbeat > HEARTBEAT_TIMEOUT_S:
            return FaultCode.HEARTBEAT_LOSS

        # ── 2. Battery ──────────────────────────────────────────────────────
        batt = self.vehicle.battery
        if batt and batt.voltage is not None:
            if batt.voltage < BATT_VOLTAGE_MIN:
                return FaultCode.CRITICAL_BATT
            if batt.level is not None and batt.level < BATT_REMAINING_MIN:
                return FaultCode.LOW_BATTERY

        # ── 3. GPS ──────────────────────────────────────────────────────────
        gps = self.vehicle.gps_0
        if gps:
            if gps.fix_type < GPS_FIX_MIN:
                return FaultCode.GPS_LOSS
            if gps.eph and gps.eph / 100.0 > GPS_HDOP_MAX:   # eph is cm*100
                return FaultCode.GPS_LOSS

        # ── 4. EKF ──────────────────────────────────────────────────────────
        ekf = self.vehicle.ekf_ok
        if ekf is False:
            return FaultCode.EKF_ERROR

        # ── 5. Attitude ─────────────────────────────────────────────────────
        att = self.vehicle.attitude
        if att:
            import math
            roll_deg  = abs(math.degrees(att.roll))
            pitch_deg = abs(math.degrees(att.pitch))
            if roll_deg > ATTITUDE_MAX_DEG or pitch_deg > ATTITUDE_MAX_DEG:
                return FaultCode.ATTITUDE_FLIP

        # ── 6. Altitude free-fall guard ──────────────────────────────────────
        # Only trigger freefall if:
        #   a) we have been airborne (alt crossed above 5 m at least once), AND
        #   b) current alt has dropped below ALT_FLOOR_M, AND
        #   c) vehicle is not in a controlled descent mode
        alt = self.vehicle.location.global_relative_frame.alt
        if alt is not None:
            if alt > 5.0:
                self._has_been_airborne = True
            if (
                self._has_been_airborne
                and alt < ALT_FLOOR_M
                and self.vehicle.mode.name not in ("LAND", "RTL", "LOITER", "STABILIZE")
            ):
                return FaultCode.FREEFALL

        return FaultCode.NONE

    # ── Fault handling ────────────────────────────────────────────────────────
    def _handle_fault(self, fault: str):
        if fault == self._fault_active:
            return  # don't spam logs for persistent faults

        self._fault_active = fault
        log.error("FAULT DETECTED: %s", fault)
        self.logger.log_fault(fault)

        if fault in DEPLOY_IMMEDIATELY:
            log.critical("DEPLOYING PARACHUTE — fault: %s", fault)
            self.parachute.deploy(reason=fault)
            self.abort_event.set()
        elif fault == FaultCode.LOW_BATTERY:
            log.warning("Low battery — signalling RTL, no parachute yet.")
            self.abort_event.set()
        elif fault == FaultCode.GPS_LOSS:
            log.warning("GPS degraded — signalling abort.")
            self.abort_event.set()