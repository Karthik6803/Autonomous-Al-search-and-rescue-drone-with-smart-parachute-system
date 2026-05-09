"""
Survivor Detector — OpenCV-based vision pipeline
Detects human-shaped targets in a camera frame using HSV colour filtering,
contour analysis, and bounding-box classification.

In simulation the "camera" is a simulated downward-facing cam via Gazebo.
For testing without a live stream, `SurvivorDetector` also supports injecting
a static test image or generating synthetic frames.
"""

import cv2
import numpy as np
import logging
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

log = logging.getLogger("VisionDetector")


# ──────────────────────────────────────────────
# Detection configuration
# ──────────────────────────────────────────────

# HSV range for "survivor marker" colour (bright orange/red vest or tarp)
HSV_LOWER_1 = np.array([0,   160,  80])   # red wrap-around low
HSV_UPPER_1 = np.array([10,  255, 255])
HSV_LOWER_2 = np.array([165, 160,  80])   # red wrap-around high
HSV_UPPER_2 = np.array([180, 255, 255])

# Secondary: bright yellow (space blanket / hi-vis)
HSV_LOWER_YELLOW = np.array([20, 120, 120])
HSV_UPPER_YELLOW = np.array([40, 255, 255])

MIN_CONTOUR_AREA   = 300   # px²  — smaller blobs are noise
MAX_CONTOUR_AREA   = 12000 # px²  — larger than this is likely terrain
ASPECT_RATIO_MIN   = 0.25  # width/height — human silhouette range
ASPECT_RATIO_MAX   = 4.0

CAMERA_SOURCE      = 0           # OpenCV VideoCapture index; change for RTSP


@dataclass
class Detection:
    bbox: Tuple[int, int, int, int]   # x, y, w, h (pixel coords)
    confidence: float                  # 0.0–1.0 heuristic
    colour_label: str                  # "red" | "yellow"
    timestamp: float


class SurvivorDetector:
    """
    Parameters
    ----------
    use_camera : bool
        True  → open a live camera (SITL Gazebo video port).
        False → generate synthetic test frames (useful without Gazebo).
    show_window : bool
        If True, open a named OpenCV window with detection overlays.
    """

    def __init__(self, use_camera: bool = False, show_window: bool = True):
        self.show_window = show_window
        self._cap: Optional[cv2.VideoCapture] = None
        self._frame_count = 0

        if use_camera:
            self._cap = cv2.VideoCapture(CAMERA_SOURCE)
            if not self._cap.isOpened():
                log.warning("Camera source %s not available — using synthetic frames.", CAMERA_SOURCE)
                self._cap = None

    # ── Public interface ─────────────────────────────────────────────────────
    def scan_frame(self) -> Optional[Detection]:
        """
        Grab one frame, run detection, return the best Detection or None.
        Renders an annotated window if `show_window` is True.
        """
        frame = self._get_frame()
        if frame is None:
            return None

        detections = self._detect(frame)
        annotated   = self._annotate(frame.copy(), detections)

        if self.show_window:
            cv2.imshow("SAR — Survivor Detection", annotated)
            cv2.waitKey(1)

        self._frame_count += 1

        if detections:
            best = max(detections, key=lambda d: d.confidence)
            log.info(
                "Detection: %s conf=%.2f bbox=%s",
                best.colour_label, best.confidence, best.bbox
            )
            return best
        return None

    def release(self):
        if self._cap:
            self._cap.release()
        cv2.destroyAllWindows()

    # ── Frame acquisition ────────────────────────────────────────────────────
    def _get_frame(self) -> Optional[np.ndarray]:
        if self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            return frame if ret else None
        # Synthetic frame for testing
        return self._synthetic_frame()

    def _synthetic_frame(self) -> np.ndarray:
        """Generate a 480×640 BGR frame; occasionally inject a fake target."""
        h, w = 480, 640
        # Textured grass background
        rng = np.random.default_rng(self._frame_count)
        frame = rng.integers(30, 80, (h, w, 3), dtype=np.uint8)
        frame[:, :, 1] = np.clip(frame[:, :, 1] + 40, 0, 255)  # greenish

        # Inject a red target every 5th frame
        if self._frame_count % 5 == 0:
            cx, cy = rng.integers(100, w - 100), rng.integers(80, h - 80)
            tw, th = rng.integers(20, 50), rng.integers(25, 60)
            # Draw red rectangle (simulating hi-vis clothing)
            frame[cy:cy+th, cx:cx+tw] = [0, 0, 200]  # BGR red
            log.debug("Injected synthetic target at (%d,%d)", cx, cy)

        return frame

    # ── Detection pipeline ───────────────────────────────────────────────────
    def _detect(self, frame: np.ndarray) -> List[Detection]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Build combined mask (red + yellow)
        mask_r1 = cv2.inRange(hsv, HSV_LOWER_1, HSV_UPPER_1)
        mask_r2 = cv2.inRange(hsv, HSV_LOWER_2, HSV_UPPER_2)
        mask_y  = cv2.inRange(hsv, HSV_LOWER_YELLOW, HSV_UPPER_YELLOW)
        colour_masks = {
            "red":    cv2.bitwise_or(mask_r1, mask_r2),
            "yellow": mask_y,
        }

        detections: List[Detection] = []

        for colour_label, mask in colour_masks.items():
            # Morphological clean-up
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (MIN_CONTOUR_AREA <= area <= MAX_CONTOUR_AREA):
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                aspect = w / max(h, 1)
                if not (ASPECT_RATIO_MIN <= aspect <= ASPECT_RATIO_MAX):
                    continue

                # Simple confidence heuristic: area normalised + roundness
                perimeter  = cv2.arcLength(cnt, True)
                roundness  = (4 * np.pi * area) / (perimeter ** 2 + 1e-6)
                confidence = min(1.0, (area / MAX_CONTOUR_AREA) * 0.6 + roundness * 0.4)

                detections.append(Detection(
                    bbox=(x, y, w, h),
                    confidence=confidence,
                    colour_label=colour_label,
                    timestamp=time.time(),
                ))

        return detections

    # ── Annotation ───────────────────────────────────────────────────────────
    def _annotate(self, frame: np.ndarray, detections: List[Detection]) -> np.ndarray:
        colour_map = {"red": (0, 0, 255), "yellow": (0, 220, 255)}

        for det in detections:
            x, y, w, h = det.bbox
            bgr = colour_map.get(det.colour_label, (255, 255, 255))
            cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, 2)
            label = f"{det.colour_label} {det.confidence:.2f}"
            cv2.putText(
                frame, label, (x, max(y - 8, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 1, cv2.LINE_AA
            )

        # HUD
        ts = time.strftime("%H:%M:%S")
        cv2.putText(
            frame, f"SAR Vision  {ts}  detected={len(detections)}",
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA
        )
        return frame


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    detector = SurvivorDetector(use_camera=False, show_window=True)
    print("Running vision test — press Q to quit")
    for _ in range(30):
        result = detector.scan_frame()
        if result:
            print(f"  → Found: {result}")
        if cv2.waitKey(100) & 0xFF == ord("q"):
            break
    detector.release()
