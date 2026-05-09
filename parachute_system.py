"""
Parachute Deployment System
Controls a parachute servo via MAVLink RC_CHANNELS_OVERRIDE / DO_SET_SERVO
and performs an engine kill on deploy.

Hardware mapping (configurable):
  • SERVO_CHANNEL  9  — parachute release (PWM 1000 = locked, 2000 = deploy)
  • SERVO_CHANNEL  10 — motor kill  (PWM 1000 = armed, 2000 = kill)

In simulation (SITL) the servo command is sent but there is no physical effect;
Gazebo can be configured to listen for the servo channel and trigger a model
joint to visualise the deployment.
"""

import logging
import time
import threading
from enum import Enum

log = logging.getLogger("Parachute")


# ──────────────────────────────────────────────
# Configuration
# ──────────────────────────────────────────────
PARACHUTE_SERVO_CHANNEL = 9     # AUX1 on Pixhawk
MOTOR_KILL_CHANNEL      = 10    # AUX2
PWM_LOCKED              = 1000
PWM_DEPLOY              = 2000
PWM_MOTOR_ARMED         = 1000
PWM_MOTOR_KILL          = 2000
DEPLOY_LOCK_SECONDS     = 5.0   # prevent re-arm for N seconds after deploy


class DeployState(Enum):
    LOCKED   = "LOCKED"
    ARMED    = "ARMED"
    DEPLOYED = "DEPLOYED"
    FAILED   = "FAILED"


class ParachuteDeploymentSystem:
    """
    Can operate standalone (no vehicle object) for testing,
    or be attached to a DroneKit Vehicle for live MAVLink commands.
    """

    def __init__(self, vehicle=None):
        self._vehicle  = vehicle
        self._state    = DeployState.LOCKED
        self._lock     = threading.Lock()
        self._deployed_at: float = 0.0
        self._deploy_count: int  = 0

    # ── Lifecycle API ─────────────────────────────────────────────────────────
    def attach_vehicle(self, vehicle):
        """Attach a DroneKit vehicle after construction."""
        self._vehicle = vehicle

    def arm_system(self):
        """Move from LOCKED → ARMED (pre-flight check passed)."""
        with self._lock:
            if self._state == DeployState.LOCKED:
                self._state = DeployState.ARMED
                log.info("Parachute system ARMED.")
            else:
                log.warning("arm_system called in state %s — ignored.", self._state)

    def lock_system(self):
        """Move back to LOCKED (post-flight or maintenance)."""
        with self._lock:
            self._state = DeployState.LOCKED
            log.info("Parachute system LOCKED.")

    @property
    def state(self) -> DeployState:
        return self._state

    # ── Deployment ────────────────────────────────────────────────────────────
    def deploy(self, reason: str = "MANUAL") -> bool:
        """
        Attempt parachute deployment.

        Returns True  on success.
        Returns False if already deployed or in locked state.
        """
        with self._lock:
            if self._state == DeployState.DEPLOYED:
                log.warning("Deploy called but already deployed — ignoring.")
                return False

            if self._state == DeployState.LOCKED:
                log.error("Deploy called while LOCKED — emergency override!")
                # Override the lock in emergency (safety-critical systems do this)

            log.critical(
                "PARACHUTE DEPLOY TRIGGERED — reason=%s  deploy#=%d",
                reason, self._deploy_count + 1
            )

            success = self._send_deploy_commands()

            if success:
                self._state       = DeployState.DEPLOYED
                self._deployed_at = time.time()
                self._deploy_count += 1
                log.critical("Parachute DEPLOYED at T=%.3f", self._deployed_at)
            else:
                self._state = DeployState.FAILED
                log.critical("Parachute deploy FAILED — no MAVLink connection.")

            return success

    # ── Low-level command sending ─────────────────────────────────────────────
    def _send_deploy_commands(self) -> bool:
        """Send servo commands via MAVLink DO_SET_SERVO."""
        if self._vehicle is None:
            log.warning("No vehicle attached — simulating deploy (SITL stub).")
            self._sim_deploy()
            return True

        try:
            from dronekit import Command
            from pymavlink import mavutil

            # Step 1: open parachute servo
            msg = self._vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                PARACHUTE_SERVO_CHANNEL,
                PWM_DEPLOY,
                0, 0, 0, 0, 0
            )
            self._vehicle.send_mavlink(msg)
            log.info("MAVLink DO_SET_SERVO ch=%d pwm=%d sent", PARACHUTE_SERVO_CHANNEL, PWM_DEPLOY)

            time.sleep(0.1)

            # Step 2: kill motors
            msg_kill = self._vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                MOTOR_KILL_CHANNEL,
                PWM_MOTOR_KILL,
                0, 0, 0, 0, 0
            )
            self._vehicle.send_mavlink(msg_kill)
            log.info("MAVLink DO_SET_SERVO ch=%d pwm=%d sent (motor kill)", MOTOR_KILL_CHANNEL, PWM_MOTOR_KILL)

            # Step 3: disarm via override (belt-and-suspenders)
            self._vehicle.armed = False

            return True

        except Exception as exc:
            log.exception("MAVLink deploy error: %s", exc)
            return False

    def _sim_deploy(self):
        """SITL / offline simulation stub — logs, writes trigger file for Gazebo bridge."""
        log.info("[SIM] Parachute servo → %d PWM", PWM_DEPLOY)
        log.info("[SIM] Motor kill servo → %d PWM", PWM_MOTOR_KILL)
        # Write trigger file so gazebo_parachute_bridge.py animates the deployment
        try:
            with open("/tmp/sar_parachute_deploy.trigger", "w") as f:
                f.write(f"DEPLOYED at {time.time():.3f}")
            log.info("[SIM] Trigger file written → /tmp/sar_parachute_deploy.trigger")
        except Exception as e:
            log.warning("Could not write trigger file: %s", e)
        time.sleep(0.05)

    # ── Status ────────────────────────────────────────────────────────────────
    def status_report(self) -> dict:
        return {
            "state":         self._state.value,
            "deploy_count":  self._deploy_count,
            "deployed_at":   self._deployed_at or None,
        }


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    chute = ParachuteDeploymentSystem()   # no vehicle — SITL stub
    chute.arm_system()
    print("State:", chute.state)
    chute.deploy(reason="TEST")
    print("Status:", chute.status_report())
