"""
Mission Logger
Records all significant events to both a structured JSON summary and a
timestamped CSV event log for post-mission analysis.

Output files (written to ./logs/):
  sar_mission_<timestamp>.json  — structured summary (survivors, faults, path)
  sar_events_<timestamp>.csv    — raw event stream (timestamp, event, data)
"""

import csv
import json
import logging
import os
import time
import threading
from datetime import datetime
from typing import List, Tuple, Optional

log = logging.getLogger("MissionLogger")

LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")


class MissionLogger:
    def __init__(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        self._json_path = os.path.join(LOG_DIR, f"sar_mission_{ts}.json")
        self._csv_path  = os.path.join(LOG_DIR, f"sar_events_{ts}.csv")
        self._lock      = threading.Lock()
        self._csv_file  = None
        self._csv_writer = None

        # In-memory accumulators
        self._start_time: float = 0.0
        self._end_time:   float = 0.0
        self._waypoints:  List[dict] = []
        self._survivors:  List[dict] = []
        self._faults:     List[dict] = []

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def start_mission(self):
        self._start_time = time.time()
        self._csv_file   = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(["epoch_s", "event", "detail"])
        self._write_csv("MISSION_START", "")
        log.info("Logging to %s", self._csv_path)

    def end_mission(self, survivors: Optional[List[Tuple]] = None):
        self._end_time = time.time()
        self._write_csv("MISSION_END", f"duration={self._end_time - self._start_time:.1f}s")

        summary = {
            "start_utc":    datetime.utcfromtimestamp(self._start_time).isoformat(),
            "end_utc":      datetime.utcfromtimestamp(self._end_time).isoformat(),
            "duration_s":   round(self._end_time - self._start_time, 1),
            "waypoints":    self._waypoints,
            "survivors":    self._survivors,
            "faults":       self._faults,
        }

        with open(self._json_path, "w") as f:
            json.dump(summary, f, indent=2)

        if self._csv_file:
            self._csv_file.close()

        log.info(
            "Mission summary → %s | survivors=%d faults=%d",
            self._json_path, len(self._survivors), len(self._faults)
        )

    # ── Event recording ───────────────────────────────────────────────────────
    def log_waypoint(self, index: int, lat: float, lon: float):
        record = {"index": index, "lat": lat, "lon": lon, "t": time.time()}
        with self._lock:
            self._waypoints.append(record)
        self._write_csv("WAYPOINT", f"{index},{lat:.6f},{lon:.6f}")

    def log_survivor(self, lat: float, lon: float):
        record = {
            "lat": lat,
            "lon": lon,
            "detected_at": datetime.utcfromtimestamp(time.time()).isoformat()
        }
        with self._lock:
            self._survivors.append(record)
        self._write_csv("SURVIVOR_DETECTED", f"{lat:.6f},{lon:.6f}")
        log.info("Survivor logged at (%.6f, %.6f)", lat, lon)

    def log_fault(self, fault_code: str):
        record = {
            "fault":     fault_code,
            "at_utc":    datetime.utcfromtimestamp(time.time()).isoformat(),
            "elapsed_s": round(time.time() - self._start_time, 1),
        }
        with self._lock:
            self._faults.append(record)
        self._write_csv("FAULT", fault_code)

    def log_event(self, tag: str, detail: str = ""):
        """Generic event hook for arbitrary annotations."""
        self._write_csv(tag, detail)

    # ── Internal ──────────────────────────────────────────────────────────────
    def _write_csv(self, event: str, detail: str):
        if self._csv_writer is None:
            return
        with self._lock:
            self._csv_writer.writerow([f"{time.time():.3f}", event, detail])
            self._csv_file.flush()
