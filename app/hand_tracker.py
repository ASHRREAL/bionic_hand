"""MediaPipe hand landmark detection and per-finger curl computation.

Curl is derived from the interior joint angles of each finger's landmark
chain: a straight finger has joint angles near 180 deg, a fully curled one
drops toward ~90 deg. The raw curl is exponentially smoothed and offset by a
user-captured baseline so the resting pose reads as zero.
"""
import logging
import math
import time

import cv2
import numpy as np

from .config_manager import FINGERS

log = logging.getLogger(__name__)

# Landmark chains: (root, joint_a, joint_b, tip). Curl uses the angles at
# joint_a and joint_b. Fingers use WRIST->MCP->PIP->TIP, thumb CMC->MCP->IP->TIP.
_CHAINS = {
    "thumb": (1, 2, 3, 4),
    "index": (0, 5, 6, 8),
    "middle": (0, 9, 10, 12),
    "ring": (0, 13, 14, 16),
    "pinky": (0, 17, 18, 20),
}

# (mean joint angle when straight, when fully curled) in degrees. These are
# approximate anatomical values; the baseline offset and dead zone absorb
# per-user variation. Tune if a finger never reaches 0.0 or 1.0.
_ANGLE_RANGE = {
    "thumb": (168.0, 130.0),
    "index": (172.0, 95.0),
    "middle": (172.0, 95.0),
    "ring": (172.0, 95.0),
    "pinky": (172.0, 95.0),
}

# Static 21-landmark skeleton (same topology as mp.solutions.hands.HAND_CONNECTIONS).
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]


def _joint_angle_deg(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """Angle at vertex b of the triangle a-b-c, in degrees."""
    v1 = a - b
    v2 = c - b
    denom = float(np.linalg.norm(v1) * np.linalg.norm(v2)) + 1e-9
    cos = float(np.dot(v1, v2)) / denom
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


class HandTracker:
    """Wraps a MediaPipe HandLandmarker (VIDEO mode) and turns landmarks into curls."""

    def __init__(self, model_path: str, min_detection_confidence: float = 0.6,
                 min_tracking_confidence: float = 0.5, smoothing: float = 0.35):
        import mediapipe as mp
        from mediapipe.tasks import python as mp_tasks
        from mediapipe.tasks.python import vision

        self._mp = mp
        options = vision.HandLandmarkerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)

        self.smoothing = max(0.01, min(1.0, smoothing))
        self._smoothed = {f: 0.0 for f in FINGERS}
        self._baseline = {f: 0.0 for f in FINGERS}
        self._last_ts_ms = 0

    def process(self, frame_bgr):
        """Run inference on one BGR frame.

        Returns (curls, landmarks): curls is {finger: 0..1} after smoothing and
        baseline correction, landmarks is the list of 21 normalized (x, y)
        points for drawing. Both are None when no hand is detected.
        """
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)

        # detect_for_video requires strictly increasing timestamps.
        ts = max(self._last_ts_ms + 1, int(time.monotonic() * 1000))
        self._last_ts_ms = ts

        result = self._landmarker.detect_for_video(mp_image, ts)
        if not result.hand_landmarks:
            return None, None

        world = result.hand_world_landmarks[0]
        points = np.array([[lm.x, lm.y, lm.z] for lm in world])

        alpha = self.smoothing
        for finger in FINGERS:
            root, ja, jb, tip = _CHAINS[finger]
            mean_angle = (
                _joint_angle_deg(points[root], points[ja], points[jb])
                + _joint_angle_deg(points[ja], points[jb], points[tip])
            ) / 2.0
            straight, curled = _ANGLE_RANGE[finger]
            raw = (straight - mean_angle) / (straight - curled)
            raw = max(0.0, min(1.0, raw))
            self._smoothed[finger] = alpha * raw + (1.0 - alpha) * self._smoothed[finger]

        curls = {}
        for finger in FINGERS:
            base = self._baseline[finger]
            adjusted = (self._smoothed[finger] - base) / max(1.0 - base, 0.05)
            curls[finger] = max(0.0, min(1.0, adjusted))

        drawable = [(lm.x, lm.y) for lm in result.hand_landmarks[0]]
        return curls, drawable

    def set_baseline(self):
        """Record the current smoothed curls as the neutral (zero) pose."""
        self._baseline = dict(self._smoothed)
        log.info("Baseline captured: %s",
                 {f: round(v, 2) for f, v in self._baseline.items()})

    def clear_baseline(self):
        self._baseline = {f: 0.0 for f in FINGERS}

    def close(self):
        self._landmarker.close()


def draw_landmarks(frame, landmarks):
    """Draw the hand skeleton on a BGR frame (landmarks are normalized x, y)."""
    h, w = frame.shape[:2]
    pts = [(int(x * w), int(y * h)) for x, y in landmarks]
    for a, b in HAND_CONNECTIONS:
        cv2.line(frame, pts[a], pts[b], (200, 200, 200), 2)
    for p in pts:
        cv2.circle(frame, p, 4, (255, 140, 0), -1)


def draw_debug_overlay(frame, curls, angles=None, fps=0.0):
    """Per-finger curl readout with a green->red progress bar, plus FPS."""
    x, y0, bar_w, bar_h, row = 10, 28, 130, 12, 30
    for i, finger in enumerate(FINGERS):
        y = y0 + i * row
        curl = curls.get(finger, 0.0) if curls else 0.0
        color = (0, int(200 * (1.0 - curl)), int(255 * curl))  # BGR green->red
        label = f"{finger:<6} {curl:4.2f}"
        if angles and finger in angles:
            label += f" {angles[finger]:3d}°"
        cv2.putText(frame, label, (x, y + bar_h - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        bx = x + 160
        cv2.rectangle(frame, (bx, y), (bx + bar_w, y + bar_h), (70, 70, 70), -1)
        cv2.rectangle(frame, (bx, y), (bx + int(bar_w * curl), y + bar_h), color, -1)
    cv2.putText(frame, f"{fps:4.1f} fps", (x, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1, cv2.LINE_AA)
