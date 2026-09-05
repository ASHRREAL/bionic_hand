"""JSON configuration: defaults, load with deep-merge, save."""
import copy
import json
import logging
from pathlib import Path

log = logging.getLogger(__name__)

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.json"
DEFAULT_MODEL_PATH = PROJECT_ROOT / "models" / "hand_landmarker.task"

DEFAULT_STRAIGHT = 10
DEFAULT_CURLED = 170

DEFAULTS = {
    "serial": {
        "port": "",
        "baud": 115200,
        # Skip sending unless at least one servo target moved by this many degrees.
        "change_threshold_deg": 2.0,
    },
    "tracking": {
        "model_path": str(DEFAULT_MODEL_PATH),
        "camera_index": 0,
        # Optional Android phone stream (IP Webcam). When set, this overrides
        # camera_index and the app streams from this URL (full-color MJPEG, no
        # v4l2loopback module needed). Leave empty to use the local webcam.
        "camera_url": "",
        # Exponential smoothing: weight of the newest sample (higher = more responsive).
        "smoothing": 0.35,
        "min_detection_confidence": 0.6,
        "min_tracking_confidence": 0.5,
    },
    "servo": {
        # Curl values within this distance of 0.0 / 1.0 snap to the endpoint.
        "dead_zone": 0.06,
        # Set true when using continuous-rotation servos with the bionic_hand_cr
        # firmware: the app then homes the hand on connect so open-loop position
        # tracking has a reference. Leave false for standard positional servos.
        "continuous_rotation": False,
    },
    "calibration": {
        f: {"straight": DEFAULT_STRAIGHT, "curled": DEFAULT_CURLED} for f in FINGERS
    },
    "debug_overlay": True,
    "first_run": True,
}


def _merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


class ConfigManager:
    def __init__(self, path=None):
        self.path = Path(path) if path else DEFAULT_CONFIG_PATH
        self.data = copy.deepcopy(DEFAULTS)

    def load(self) -> dict:
        if self.path.exists():
            try:
                loaded = json.loads(self.path.read_text())
                self.data = _merge(DEFAULTS, loaded)
                log.info("Loaded config from %s", self.path)
            except (json.JSONDecodeError, OSError) as exc:
                log.error("Could not read %s (%s) — using defaults", self.path, exc)
                self.data = copy.deepcopy(DEFAULTS)
        else:
            log.info("No config file at %s — creating one with defaults", self.path)
            self.save()
        return self.data

    def save(self) -> bool:
        try:
            self.path.write_text(json.dumps(self.data, indent=2) + "\n")
            log.info("Saved config to %s", self.path)
            return True
        except OSError as exc:
            log.error("Could not write %s: %s", self.path, exc)
            return False
