"""Camera capture thread: frames -> MediaPipe -> curls -> servo angles -> serial.

The worker never touches Tk widgets. It publishes an annotated frame plus the
latest curls/angles into a lock-protected snapshot; the GUI polls the snapshot
from the Tk thread with after(). Shared mode/debug flags live in AppState.
"""
import logging
import threading
import time

import cv2

from .config_manager import FINGERS
from .hand_tracker import HandTracker, draw_landmarks, draw_debug_overlay

log = logging.getLogger(__name__)

MODE_TRACK = "track"
MODE_MANUAL = "manual"
MODE_CALIBRATE = "calibrate"


class AppState:
    """Thread-safe flags shared between the GUI and the capture thread."""

    def __init__(self, mode=MODE_TRACK, debug_overlay=True):
        self._lock = threading.Lock()
        self._mode = mode
        self._debug = debug_overlay

    @property
    def mode(self):
        with self._lock:
            return self._mode

    @mode.setter
    def mode(self, value):
        with self._lock:
            self._mode = value

    @property
    def debug_overlay(self):
        with self._lock:
            return self._debug

    @debug_overlay.setter
    def debug_overlay(self, value):
        with self._lock:
            self._debug = bool(value)


class CaptureWorker(threading.Thread):
    CAMERA_RETRY_S = 3.0

    def __init__(self, cfg: dict, state: AppState, mapper, link):
        super().__init__(daemon=True, name="capture")
        self._cfg = cfg
        self._state = state
        self._mapper = mapper
        self._link = link
        self._stop_evt = threading.Event()
        self._baseline_evt = threading.Event()
        self._snap_lock = threading.Lock()
        self._snapshot = {
            "frame": None,      # annotated BGR frame (np.ndarray)
            "curls": None,      # {finger: 0..1} or None
            "angles": None,     # {finger: deg} or None
            "hand": False,
            "error": None,      # user-facing problem string or None
            "fps": 0.0,
        }

    # ---------------- GUI-facing API ----------------

    def get_snapshot(self) -> dict:
        with self._snap_lock:
            return dict(self._snapshot)

    def request_baseline(self):
        """Capture the current hand pose as neutral on the next tracked frame."""
        self._baseline_evt.set()

    def stop(self):
        self._stop_evt.set()

    # ---------------- worker internals ----------------

    def _publish(self, **fields):
        with self._snap_lock:
            self._snapshot.update(fields)

    def _make_tracker(self):
        t = self._cfg["tracking"]
        try:
            return HandTracker(
                model_path=t["model_path"],
                min_detection_confidence=t["min_detection_confidence"],
                min_tracking_confidence=t["min_tracking_confidence"],
                smoothing=t["smoothing"],
            ), None
        except Exception as exc:
            log.error("Hand tracker init failed: %s", exc)
            return None, (f"Hand model unavailable ({exc}).\n"
                          "Run setup.sh (or the curl command in README.md) "
                          "to download models/hand_landmarker.task")

    def _open_camera(self):
        trk = self._cfg["tracking"]
        # A phone IP Webcam stream (HTTP MJPEG) takes priority when configured.
        url = (trk.get("camera_url") or "").strip()
        if url:
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            if cap.isOpened():
                return cap
            cap.release()
            log.error("Could not open camera_url %s", url)
            return None
        index = trk.get("camera_index", 0)
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            return cap
        cap.release()
        return None

    def run(self):
        tracker, tracker_error = self._make_tracker()
        cap = None
        last_frame_t = time.monotonic()
        fps = 0.0

        try:
            while not self._stop_evt.is_set():
                if cap is None:
                    cap = self._open_camera()
                    if cap is None:
                        self._publish(frame=None, hand=False,
                                      error="Camera not found — check that a webcam "
                                            "is plugged in and not in use")
                        if self._stop_evt.wait(self.CAMERA_RETRY_S):
                            break
                        continue

                ok, frame = cap.read()
                if not ok or frame is None:
                    cap.release()
                    cap = None
                    continue
                frame = cv2.flip(frame, 1)  # mirror view feels natural

                now = time.monotonic()
                dt = now - last_frame_t
                last_frame_t = now
                if dt > 0:
                    fps = 0.9 * fps + 0.1 * (1.0 / dt)

                curls, landmarks = None, None
                if tracker is not None:
                    try:
                        curls, landmarks = tracker.process(frame)
                    except Exception as exc:
                        log.exception("Tracking failed on frame")
                        tracker_error = f"Tracking error: {exc}"
                        tracker = None

                if curls is not None and self._baseline_evt.is_set():
                    tracker.set_baseline()
                    self._baseline_evt.clear()

                angles = self._mapper.map_all(curls) if curls else None
                if angles and self._state.mode == MODE_TRACK:
                    self._link.send_angles(angles)

                if landmarks:
                    draw_landmarks(frame, landmarks)
                if self._state.debug_overlay:
                    draw_debug_overlay(frame, curls, angles, fps)

                self._publish(frame=frame, curls=curls, angles=angles,
                              hand=curls is not None, error=tracker_error, fps=fps)
        finally:
            if cap is not None:
                cap.release()
            if tracker is not None:
                tracker.close()
            log.info("Capture worker stopped")
