"""Maps curl values (0..1) to servo angles via per-finger calibration."""
import copy
import logging
import threading

from .config_manager import FINGERS, DEFAULT_STRAIGHT, DEFAULT_CURLED

log = logging.getLogger(__name__)


class ServoMapper:
    """curl 0.0 -> straight angle, 1.0 -> curled angle, linear in between.

    A dead zone at both extremes snaps near-endpoint curls to the endpoint
    (stops endpoint jitter) and rescales the middle so the mapping stays
    continuous. Shared between the GUI and capture threads, so calibration
    access is guarded by a lock.
    """

    def __init__(self, calibration: dict, dead_zone: float = 0.06):
        self._lock = threading.Lock()
        self._cal = {}
        for finger in FINGERS:
            entry = calibration.get(finger, {})
            self._cal[finger] = {
                "straight": int(entry.get("straight", DEFAULT_STRAIGHT)),
                "curled": int(entry.get("curled", DEFAULT_CURLED)),
            }
        self.dead_zone = max(0.0, min(0.4, dead_zone))

    def curl_to_angle(self, finger: str, curl: float) -> int:
        curl = max(0.0, min(1.0, curl))
        dz = self.dead_zone
        if curl < dz:
            curl = 0.0
        elif curl > 1.0 - dz:
            curl = 1.0
        elif dz > 0.0:
            curl = (curl - dz) / (1.0 - 2.0 * dz)
        with self._lock:
            straight = self._cal[finger]["straight"]
            curled = self._cal[finger]["curled"]
        return int(round(straight + curl * (curled - straight)))

    def map_all(self, curls: dict) -> dict:
        return {f: self.curl_to_angle(f, curls[f]) for f in FINGERS if f in curls}

    def set_finger(self, finger: str, straight=None, curled=None):
        with self._lock:
            if straight is not None:
                self._cal[finger]["straight"] = max(0, min(180, int(straight)))
            if curled is not None:
                self._cal[finger]["curled"] = max(0, min(180, int(curled)))

    def get_calibration(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._cal)

    def set_calibration(self, calibration: dict):
        for finger in FINGERS:
            entry = calibration.get(finger)
            if entry:
                self.set_finger(finger, entry.get("straight"), entry.get("curled"))

    def reset_defaults(self):
        for finger in FINGERS:
            self.set_finger(finger, DEFAULT_STRAIGHT, DEFAULT_CURLED)
        log.info("Calibration reset to defaults (%d/%d)", DEFAULT_STRAIGHT, DEFAULT_CURLED)
