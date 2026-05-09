"""
SAR Drone — Parachute Demo Trigger
====================================
Run this while the mission is flying to inject a fault and
demonstrate parachute deployment on demand.

Usage:
    python trigger_parachute_demo.py [fault_type]

Fault types:
    battery     — simulate critical battery failure
    attitude    — simulate flip / loss of control
    freefall    — simulate hardware failure / freefall
    gps         — simulate GPS loss
    manual      — manual deployment (default)

Example:
    python trigger_parachute_demo.py attitude
"""

import sys
import time
import os

TRIGGER_FILE = "/tmp/sar_parachute_deploy.trigger"

FAULTS = {
    "battery":  "CRITICAL_BATTERY",
    "attitude": "ATTITUDE_FLIP",
    "freefall": "FREEFALL",
    "gps":      "GPS_LOSS",
    "manual":   "MANUAL_DEMO",
}

RED    = "\033[91m"
YELLOW = "\033[93m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def main():
    fault_key = sys.argv[1] if len(sys.argv) > 1 else "manual"
    fault_code = FAULTS.get(fault_key, "MANUAL_DEMO")

    print(f"\n{RED}{BOLD}{'═'*50}{RESET}")
    print(f"{RED}{BOLD}  INJECTING FAULT: {fault_code}{RESET}")
    print(f"{RED}{BOLD}{'═'*50}{RESET}\n")

    # Write the trigger file
    with open(TRIGGER_FILE, "w") as f:
        f.write(fault_code)

    print(f"{YELLOW}Trigger written → {TRIGGER_FILE}{RESET}")
    print(f"{YELLOW}Parachute bridge will now animate deployment in Gazebo.{RESET}\n")

    # Also directly invoke the parachute system for the mission
    try:
        from parachute_system import ParachuteDeploymentSystem
        chute = ParachuteDeploymentSystem()
        chute.arm_system()
        chute.deploy(reason=fault_code)
        print(f"Parachute system status: {chute.status_report()}")
    except Exception as e:
        print(f"(Parachute system direct call skipped: {e})")

if __name__ == "__main__":
    main()
