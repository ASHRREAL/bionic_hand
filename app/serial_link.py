"""Thread-safe serial link to the ESP32 with heartbeat and auto-reconnect.

Protocol (newline-delimited ASCII, 115200 baud):
    S,thumb:90,index:45,...        set target angles
    P            -> device replies PONG
    X            emergency stop (device detaches all servos)
    C,thumb:20,160,index:10,170,...  push calibration bounds

A single manager thread owns the port lifecycle: it opens the port, reads
replies, sends a heartbeat ping every HEARTBEAT_PERIOD seconds, and if the
port errors out or the device stops answering pings, reconnects with
exponential backoff. All writes (from any thread) go through send_line(),
which holds an RLock around the port.
"""
import logging
import threading
import time
from enum import Enum

import serial
from serial.tools import list_ports as _list_ports

from .config_manager import FINGERS

log = logging.getLogger(__name__)


class LinkState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"


class SerialLink:
    HEARTBEAT_PERIOD = 2.0   # seconds between P pings
    PONG_TIMEOUT = 6.0       # no PONG for this long -> connection considered dead
    MAX_BACKOFF = 30.0       # cap for reconnect delay
    BOOT_DELAY = 1.5         # ESP32 dev boards reset when the UART bridge opens

    def __init__(self, baud=115200, change_threshold=2.0,
                 on_state=None, on_connected=None):
        """on_state(state, detail) and on_connected() may fire from the manager
        thread — GUI callers must marshal them onto the Tk thread themselves."""
        self._baud = baud
        self._threshold = change_threshold
        self._on_state = on_state
        self._on_connected = on_connected

        self._ser = None
        self._io_lock = threading.RLock()
        self._state = LinkState.DISCONNECTED
        self._detail = ""
        self._port = None
        self._want = False        # user wants a connection kept alive
        self._closing = False
        self._write_failed = False
        self._last_sent = {}
        self._wake = threading.Event()
        self._thread = None

    @staticmethod
    def list_ports():
        return [p.device for p in _list_ports.comports()]

    @property
    def state(self):
        return self._state

    @property
    def is_connected(self):
        return self._state == LinkState.CONNECTED

    # ---------------- public API ----------------

    def connect(self, port: str):
        self._port = port
        self._want = True
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._manager, daemon=True,
                                            name="serial-manager")
            self._thread.start()
        self._wake.set()

    def disconnect(self):
        self._want = False
        self._wake.set()

    def close(self):
        self._closing = True
        self._want = False
        self._wake.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)

    def send_line(self, text: str) -> bool:
        with self._io_lock:
            if self._ser is None:
                return False
            try:
                self._ser.write((text + "\n").encode("ascii"))
                return True
            except (serial.SerialException, OSError) as exc:
                log.warning("Serial write failed: %s", exc)
                self._write_failed = True
                self._wake.set()
                return False

    def send_angles(self, angles: dict, force=False) -> bool:
        """Send an S command. Unless force, skip if no servo moved by more
        than the change threshold since the last successful send."""
        ints = {f: max(0, min(180, int(round(angles[f]))))
                for f in FINGERS if f in angles}
        if not ints:
            return False
        if not force and self._last_sent:
            changed = any(abs(ints[f] - self._last_sent.get(f, 9999)) >= self._threshold
                          for f in ints)
            if not changed:
                return False
        cmd = "S," + ",".join(f"{f}:{ints[f]}" for f in FINGERS if f in ints)
        if self.send_line(cmd):
            self._last_sent.update(ints)
            return True
        return False

    def send_stop(self) -> bool:
        # Clear the dedup cache so the next S command always goes out and
        # re-attaches the servos on the device.
        self._last_sent = {}
        return self.send_line("X")

    def send_calibration(self, calibration: dict) -> bool:
        parts = [f"{f}:{calibration[f]['straight']},{calibration[f]['curled']}"
                 for f in FINGERS if f in calibration]
        return self.send_line("C," + ",".join(parts))

    def send_home(self) -> bool:
        # Continuous-rotation firmware only; positional firmware ignores it.
        self._last_sent = {}  # positions change after homing; force next S
        return self.send_line("H")

    # ---------------- manager thread ----------------

    def _set_state(self, state, detail=""):
        if state == self._state and detail == self._detail:
            return
        self._state, self._detail = state, detail
        log.info("Serial link: %s %s", state.value, detail)
        if self._on_state:
            try:
                self._on_state(state, detail)
            except Exception:
                log.exception("on_state callback failed")

    def _teardown_port(self):
        with self._io_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
            self._write_failed = False

    def _wait(self, seconds):
        self._wake.wait(timeout=seconds)
        self._wake.clear()

    def _manager(self):
        backoff = 1.0
        last_hb = 0.0
        last_pong = 0.0
        while not self._closing:
            if not self._want:
                self._teardown_port()
                self._set_state(LinkState.DISCONNECTED)
                self._wait(0.5)
                continue

            if self._ser is None:
                self._set_state(LinkState.CONNECTING, f"opening {self._port}")
                try:
                    ser = serial.Serial(self._port, self._baud,
                                        timeout=0.2, write_timeout=1.0)
                except (serial.SerialException, OSError, ValueError) as exc:
                    self._set_state(LinkState.CONNECTING,
                                    f"open failed, retry in {backoff:.0f}s: {exc}")
                    self._wait(backoff)
                    backoff = min(backoff * 2.0, self.MAX_BACKOFF)
                    continue
                time.sleep(self.BOOT_DELAY)
                with self._io_lock:
                    self._ser = ser
                    try:
                        self._ser.reset_input_buffer()
                    except (serial.SerialException, OSError):
                        pass
                self._last_sent = {}
                last_pong = time.monotonic()
                last_hb = 0.0
                backoff = 1.0
                self._set_state(LinkState.CONNECTED, self._port)
                if self._on_connected:
                    try:
                        self._on_connected()
                    except Exception:
                        log.exception("on_connected callback failed")

            try:
                if self._write_failed:
                    raise serial.SerialException("write failed")
                line = self._ser.readline()  # returns b"" after timeout
                if line:
                    text = line.decode("ascii", errors="replace").strip()
                    if text == "PONG":
                        last_pong = time.monotonic()
                    elif text:
                        log.debug("Device: %s", text)
                now = time.monotonic()
                if now - last_hb >= self.HEARTBEAT_PERIOD:
                    last_hb = now
                    self.send_line("P")
                if now - last_pong > self.PONG_TIMEOUT:
                    raise serial.SerialException("heartbeat timed out (no PONG)")
            except (serial.SerialException, OSError) as exc:
                log.warning("Serial link lost: %s", exc)
                self._teardown_port()
                if self._want and not self._closing:
                    self._set_state(LinkState.CONNECTING,
                                    f"reconnecting in {backoff:.0f}s")
                    self._wait(backoff)
                    backoff = min(backoff * 2.0, self.MAX_BACKOFF)

        self._teardown_port()
        self._set_state(LinkState.DISCONNECTED)
