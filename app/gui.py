"""CustomTkinter GUI for the bionic hand controller.

Threading contract: the capture worker and serial manager threads never touch
widgets. Worker output is polled from the Tk thread via after() loops; serial
callbacks (which fire on the manager thread) enqueue closures onto a
queue.Queue that the Tk thread drains.
"""
import logging
import queue
import tkinter as tk
import tkinter.messagebox as messagebox

import cv2
import customtkinter as ctk
from PIL import Image

from .capture_worker import (AppState, CaptureWorker,
                             MODE_CALIBRATE, MODE_MANUAL, MODE_TRACK)
from .config_manager import FINGERS, ConfigManager
from .serial_link import LinkState, SerialLink
from .servo_mapper import ServoMapper

log = logging.getLogger(__name__)

VIDEO_W, VIDEO_H = 640, 480
FRAME_POLL_MS = 33
STATUS_POLL_MS = 100

STATE_COLORS = {
    LinkState.CONNECTED: "#2ecc71",
    LinkState.CONNECTING: "#f1c40f",
    LinkState.DISCONNECTED: "#e74c3c",
}
MODE_LABELS = {"Track": MODE_TRACK, "Manual": MODE_MANUAL, "Calibrate": MODE_CALIBRATE}
NO_PORTS = "(no ports found)"


def _parse_angle(text, fallback):
    try:
        return max(0, min(180, int(float(text))))
    except (ValueError, TypeError):
        return fallback


class ServoCard(ctk.CTkFrame):
    """One finger: angle readout, slider, calibration fields, jog buttons."""

    def __init__(self, master, finger: str, app: "BionicHandApp"):
        super().__init__(master, corner_radius=10)
        self.finger = finger
        self.app = app

        self.grid_columnconfigure(0, weight=1)

        header = ctk.CTkFrame(self, fg_color="transparent")
        header.grid(row=0, column=0, sticky="ew", padx=10, pady=(8, 0))
        ctk.CTkLabel(header, text=finger.capitalize(),
                     font=ctk.CTkFont(size=15, weight="bold")).pack(side="left")
        self.jog_frame = ctk.CTkFrame(header, fg_color="transparent")
        ctk.CTkButton(self.jog_frame, text="-1°", width=36,
                      command=lambda: app.on_jog(finger, -1)).pack(side="left", padx=2)
        ctk.CTkButton(self.jog_frame, text="+1°", width=36,
                      command=lambda: app.on_jog(finger, +1)).pack(side="left", padx=2)
        # packed/unpacked by show_jog()

        self.angle_label = ctk.CTkLabel(self, text="90°",
                                        font=ctk.CTkFont(size=30, weight="bold"))
        self.angle_label.grid(row=1, column=0, pady=(2, 0))

        self.slider = ctk.CTkSlider(self, from_=0, to=180, number_of_steps=180,
                                    command=self._on_slider)
        self.slider.set(90)
        self.slider.grid(row=2, column=0, sticky="ew", padx=12, pady=(2, 6))

        cal = ctk.CTkFrame(self, fg_color="transparent")
        cal.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 8))
        cal.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(cal, text="Straight", width=54, anchor="w").grid(row=0, column=0)
        self.straight_entry = ctk.CTkEntry(cal, width=52, justify="center")
        self.straight_entry.grid(row=0, column=1, sticky="w", padx=4)
        ctk.CTkButton(cal, text="Set Straight", width=90,
                      command=lambda: app.on_snapshot(finger, "straight")
                      ).grid(row=0, column=2, padx=2, pady=1)

        ctk.CTkLabel(cal, text="Curled", width=54, anchor="w").grid(row=1, column=0)
        self.curled_entry = ctk.CTkEntry(cal, width=52, justify="center")
        self.curled_entry.grid(row=1, column=1, sticky="w", padx=4)
        ctk.CTkButton(cal, text="Set Curled", width=90,
                      command=lambda: app.on_snapshot(finger, "curled")
                      ).grid(row=1, column=2, padx=2, pady=1)

        for entry in (self.straight_entry, self.curled_entry):
            entry.bind("<Return>", lambda _e: app.on_cal_entry_commit(finger))
            entry.bind("<FocusOut>", lambda _e: app.on_cal_entry_commit(finger))

    def _on_slider(self, value):
        self.set_angle_display(int(value))
        self.app.on_slider_moved(self.finger)

    def set_angle_display(self, angle: int):
        self.angle_label.configure(text=f"{int(angle)}°")

    def set_slider_silent(self, angle: int):
        """Move the slider without emitting a manual-send (CTkSlider.set does
        not invoke the command callback)."""
        self.slider.set(int(angle))
        self.set_angle_display(int(angle))

    def set_calibration_fields(self, straight: int, curled: int):
        for entry, value in ((self.straight_entry, straight),
                             (self.curled_entry, curled)):
            entry.delete(0, "end")
            entry.insert(0, str(int(value)))

    def get_calibration_fields(self, fallback: dict):
        return (_parse_angle(self.straight_entry.get(), fallback["straight"]),
                _parse_angle(self.curled_entry.get(), fallback["curled"]))

    def show_jog(self, visible: bool):
        if visible:
            self.jog_frame.pack(side="right")
        else:
            self.jog_frame.pack_forget()

    def set_slider_enabled(self, enabled: bool):
        self.slider.configure(state="normal" if enabled else "disabled")


class CalibrationWizard(ctk.CTkToplevel):
    """Steps through straight/curled capture for each finger."""

    JOGS = (-10, -1, +1, +10)

    def __init__(self, app: "BionicHandApp"):
        super().__init__(app)
        self.app = app
        self.title("Calibration Wizard")
        self.geometry("460x330")
        self.resizable(False, False)
        self.steps = [(f, phase) for f in FINGERS for phase in ("straight", "curled")]
        self.index = 0
        self.angle = 90

        self.step_label = ctk.CTkLabel(self, text="", font=ctk.CTkFont(size=18, weight="bold"))
        self.step_label.pack(pady=(18, 4))
        self.instruction = ctk.CTkLabel(self, text="", wraplength=400, justify="center")
        self.instruction.pack(pady=(0, 10))
        self.angle_label = ctk.CTkLabel(self, text="90°", font=ctk.CTkFont(size=34, weight="bold"))
        self.angle_label.pack(pady=4)

        jog_row = ctk.CTkFrame(self, fg_color="transparent")
        jog_row.pack(pady=6)
        for delta in self.JOGS:
            ctk.CTkButton(jog_row, text=f"{delta:+d}°", width=56,
                          command=lambda d=delta: self._jog(d)).pack(side="left", padx=4)

        btn_row = ctk.CTkFrame(self, fg_color="transparent")
        btn_row.pack(pady=16)
        ctk.CTkButton(btn_row, text="Capture", width=110,
                      fg_color="#2e8b57", command=self._capture).pack(side="left", padx=6)
        ctk.CTkButton(btn_row, text="Skip", width=80,
                      command=self._next).pack(side="left", padx=6)
        ctk.CTkButton(btn_row, text="Cancel", width=80, fg_color="#8b3a3a",
                      command=self.destroy).pack(side="left", padx=6)

        self._load_step()
        self.grab_set()

    def _current(self):
        return self.steps[self.index]

    def _load_step(self):
        finger, phase = self._current()
        cal = self.app.mapper.get_calibration()[finger]
        self.angle = cal[phase]
        self.step_label.configure(
            text=f"{finger.capitalize()} — {phase.upper()}  "
                 f"({self.index + 1}/{len(self.steps)})")
        pose = ("fully STRAIGHT / open" if phase == "straight"
                else "fully CURLED / closed")
        self.instruction.configure(
            text=f"Jog the servo until the prosthetic {finger} is {pose}, "
                 "then press Capture.")
        self._show_angle(send=True)

    def _show_angle(self, send=False):
        self.angle_label.configure(text=f"{self.angle}°")
        if send:
            finger, _ = self._current()
            self.app.serial.send_angles({finger: self.angle}, force=True)

    def _jog(self, delta):
        self.angle = max(0, min(180, self.angle + delta))
        self._show_angle(send=True)

    def _capture(self):
        finger, phase = self._current()
        self.app.apply_wizard_value(finger, phase, self.angle)
        self._next()

    def _next(self):
        self.index += 1
        if self.index >= len(self.steps):
            self.app.save_calibration()
            self.app.set_status("Wizard complete — calibration saved")
            self.destroy()
        else:
            self._load_step()


class BionicHandApp(ctk.CTk):
    def __init__(self, cfg_mgr: ConfigManager):
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        super().__init__()
        self.title("Bionic Hand Controller")
        self.geometry("1280x760")
        self.minsize(1100, 680)

        self.cfg_mgr = cfg_mgr
        cfg = cfg_mgr.data

        self._continuous_rotation = cfg["servo"].get("continuous_rotation", False)
        self.state_flags = AppState(mode=MODE_TRACK,
                                    debug_overlay=cfg["debug_overlay"])
        self.mapper = ServoMapper(cfg["calibration"],
                                  dead_zone=cfg["servo"]["dead_zone"])
        self.serial = SerialLink(
            baud=cfg["serial"]["baud"],
            change_threshold=cfg["serial"]["change_threshold_deg"],
            on_state=self._on_serial_state_threaded,
            on_connected=self._on_serial_connected_threaded,
        )
        self.worker = CaptureWorker(cfg, self.state_flags, self.mapper, self.serial)

        self._ui_queue: "queue.Queue" = queue.Queue()
        self._manual_send_job = None
        self._video_image = None  # keep a reference or Tk drops the image
        self.cards: dict[str, ServoCard] = {}

        self._build_top_bar()
        self._build_main_area()
        self._build_bottom_bar()
        self._apply_calibration_to_cards()
        self._apply_mode("Track")

        self.protocol("WM_DELETE_WINDOW", self.on_quit)
        self.worker.start()
        self.after(FRAME_POLL_MS, self._poll_frame)
        self.after(STATUS_POLL_MS, self._poll_ui_queue)
        self.after(600, self._maybe_first_run)

    # ---------------- layout ----------------

    def _build_top_bar(self):
        bar = ctk.CTkFrame(self, corner_radius=0)
        bar.pack(side="top", fill="x")

        self.port_menu = ctk.CTkOptionMenu(bar, values=[NO_PORTS], width=210)
        self.port_menu.pack(side="left", padx=(12, 4), pady=8)
        ctk.CTkButton(bar, text="⟳", width=32,
                      command=self.refresh_ports).pack(side="left", padx=2)
        self.connect_btn = ctk.CTkButton(bar, text="Connect", width=110,
                                         command=self.on_connect_toggle)
        self.connect_btn.pack(side="left", padx=6)
        self.status_dot = ctk.CTkLabel(bar, text="●", width=24,
                                       font=ctk.CTkFont(size=20),
                                       text_color=STATE_COLORS[LinkState.DISCONNECTED])
        self.status_dot.pack(side="left")

        self.mode_selector = ctk.CTkSegmentedButton(
            bar, values=list(MODE_LABELS), command=self._apply_mode)
        self.mode_selector.set("Track")
        self.mode_selector.pack(side="left", padx=18)

        ctk.CTkButton(bar, text="Reset Baseline", width=120,
                      command=self.on_reset_baseline).pack(side="left", padx=6)

        self.debug_switch = ctk.CTkSwitch(bar, text="Show Debug",
                                          command=self._on_debug_toggle)
        if self.cfg_mgr.data["debug_overlay"]:
            self.debug_switch.select()
        self.debug_switch.pack(side="right", padx=12)

        self.refresh_ports()

    def _build_main_area(self):
        main = ctk.CTkFrame(self, fg_color="transparent")
        main.pack(side="top", fill="both", expand=True, padx=10, pady=8)
        main.grid_columnconfigure(1, weight=1)
        main.grid_rowconfigure(0, weight=1)

        video_panel = ctk.CTkFrame(main, corner_radius=10, width=VIDEO_W + 20)
        video_panel.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        video_panel.grid_propagate(False)
        self.video_label = ctk.CTkLabel(video_panel, text="Starting camera…",
                                        width=VIDEO_W, height=VIDEO_H)
        self.video_label.pack(expand=True, padx=10, pady=10)

        grid = ctk.CTkFrame(main, fg_color="transparent")
        grid.grid(row=0, column=1, sticky="nsew")
        for col in range(3):
            grid.grid_columnconfigure(col, weight=1, uniform="cards")
        for row in range(2):
            grid.grid_rowconfigure(row, weight=1)
        positions = [(0, 0), (0, 1), (0, 2), (1, 0), (1, 1)]  # 3x2, last cell empty
        for finger, (row, col) in zip(FINGERS, positions):
            card = ServoCard(grid, finger, self)
            card.grid(row=row, column=col, sticky="nsew", padx=5, pady=5)
            self.cards[finger] = card

    def _build_bottom_bar(self):
        bar = ctk.CTkFrame(self, corner_radius=0)
        bar.pack(side="bottom", fill="x")

        ctk.CTkButton(bar, text="Save Calibration", width=130,
                      command=self.save_calibration).pack(side="left", padx=(12, 4), pady=8)
        ctk.CTkButton(bar, text="Load Calibration", width=130,
                      command=self.load_calibration).pack(side="left", padx=4)
        ctk.CTkButton(bar, text="Reset to Defaults", width=130,
                      command=self.reset_calibration).pack(side="left", padx=4)
        ctk.CTkButton(bar, text="Calibration Wizard", width=140,
                      command=self.open_wizard).pack(side="left", padx=4)
        if self._continuous_rotation:
            ctk.CTkButton(bar, text="Home", width=70,
                          command=self.on_home).pack(side="left", padx=4)

        self.status_label = ctk.CTkLabel(bar, text="Ready", anchor="w")
        self.status_label.pack(side="left", fill="x", expand=True, padx=16)

        ctk.CTkButton(bar, text="QUIT", width=80, fg_color="#5a5a5a",
                      command=self.on_quit).pack(side="right", padx=(4, 12))
        ctk.CTkButton(bar, text="STOP", width=90, fg_color="#c0392b",
                      hover_color="#96281b",
                      command=self.on_stop).pack(side="right", padx=4)
        ctk.CTkButton(bar, text="Test 90°", width=90,
                      command=self.on_test_90).pack(side="right", padx=4)

    # ---------------- status / marshalling ----------------

    def set_status(self, message: str):
        self.status_label.configure(text=message)
        log.info("Status: %s", message)

    def _on_serial_state_threaded(self, state, detail):
        self._ui_queue.put(lambda: self._on_serial_state(state, detail))

    def _on_serial_connected_threaded(self):
        self._ui_queue.put(self._on_serial_connected)

    def _on_serial_state(self, state, detail):
        self.status_dot.configure(text_color=STATE_COLORS[state])
        if state == LinkState.DISCONNECTED:
            self.connect_btn.configure(text="Connect")
            self.port_menu.configure(state="normal")
            self.set_status("Serial disconnected")
        elif state == LinkState.CONNECTING:
            self.connect_btn.configure(text="Cancel")
            self.port_menu.configure(state="disabled")
            self.set_status(f"Connecting… {detail}")
        else:
            self.connect_btn.configure(text="Disconnect")
            self.port_menu.configure(state="disabled")
            self.set_status(f"Connected to {detail}")

    def _on_serial_connected(self):
        # Push calibration bounds so the ESP32 can clamp locally.
        self.serial.send_calibration(self.mapper.get_calibration())
        if self._continuous_rotation:
            self.serial.send_home()
            self.set_status("Connected — calibration pushed, homing hand…")
        else:
            self.set_status("Connected — calibration pushed to device")

    def _poll_ui_queue(self):
        try:
            while True:
                self._ui_queue.get_nowait()()
        except queue.Empty:
            pass
        self.after(STATUS_POLL_MS, self._poll_ui_queue)

    # ---------------- video / tracking updates ----------------

    def _poll_frame(self):
        snap = self.worker.get_snapshot()
        frame = snap["frame"]
        if frame is not None:
            h, w = frame.shape[:2]
            scale = min(VIDEO_W / w, VIDEO_H / h)
            size = (int(w * scale), int(h * scale))
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil = Image.fromarray(rgb)
            self._video_image = ctk.CTkImage(light_image=pil, dark_image=pil, size=size)
            text = snap["error"] or ""
            self.video_label.configure(image=self._video_image, text=text)
        else:
            self.video_label.configure(
                image=None, text=snap["error"] or "Waiting for camera…")

        if self.state_flags.mode == MODE_TRACK and snap["angles"]:
            for finger, angle in snap["angles"].items():
                self.cards[finger].set_slider_silent(angle)

        self.after(FRAME_POLL_MS, self._poll_frame)

    # ---------------- top bar handlers ----------------

    def refresh_ports(self):
        ports = SerialLink.list_ports()
        values = ports or [NO_PORTS]
        self.port_menu.configure(values=values)
        remembered = self.cfg_mgr.data["serial"]["port"]
        if remembered in ports:
            self.port_menu.set(remembered)
        else:
            self.port_menu.set(values[0])

    def on_connect_toggle(self):
        if self.serial.state != LinkState.DISCONNECTED:
            self.serial.disconnect()
            return
        port = self.port_menu.get()
        if not port or port == NO_PORTS:
            self.set_status("No serial port selected — plug in the ESP32 and press ⟳")
            return
        self.cfg_mgr.data["serial"]["port"] = port
        self.cfg_mgr.save()
        self.serial.connect(port)

    def _apply_mode(self, label: str):
        mode = MODE_LABELS[label]
        self.state_flags.mode = mode
        sliders_enabled = mode in (MODE_MANUAL, MODE_CALIBRATE)
        for card in self.cards.values():
            card.set_slider_enabled(sliders_enabled)
            card.show_jog(mode == MODE_CALIBRATE)
        self.set_status(f"{label} mode")

    def on_reset_baseline(self):
        self.worker.request_baseline()
        self.set_status("Baseline: hold your hand relaxed in view — capturing…")

    def _on_debug_toggle(self):
        enabled = bool(self.debug_switch.get())
        self.state_flags.debug_overlay = enabled
        self.cfg_mgr.data["debug_overlay"] = enabled

    # ---------------- servo card handlers ----------------

    def on_slider_moved(self, finger: str):
        if self.state_flags.mode == MODE_TRACK:
            return  # tracking owns the servos; slider is display-only
        self._queue_manual_send()

    def _queue_manual_send(self):
        if self._manual_send_job is None:
            self._manual_send_job = self.after(60, self._do_manual_send)

    def _do_manual_send(self):
        self._manual_send_job = None
        angles = {f: int(card.slider.get()) for f, card in self.cards.items()}
        self.serial.send_angles(angles, force=True)

    def on_jog(self, finger: str, delta: int):
        card = self.cards[finger]
        angle = max(0, min(180, int(card.slider.get()) + delta))
        card.set_slider_silent(angle)
        self._queue_manual_send()

    def on_snapshot(self, finger: str, phase: str):
        """Set Straight / Set Curled: snapshot the slider into calibration."""
        angle = int(self.cards[finger].slider.get())
        self.mapper.set_finger(finger, **{phase: angle})
        cal = self.mapper.get_calibration()[finger]
        self.cards[finger].set_calibration_fields(cal["straight"], cal["curled"])
        self.set_status(f"{finger.capitalize()} {phase} = {angle}° "
                        "(press Save Calibration to persist)")

    def on_cal_entry_commit(self, finger: str):
        card = self.cards[finger]
        current = self.mapper.get_calibration()[finger]
        straight, curled = card.get_calibration_fields(current)
        self.mapper.set_finger(finger, straight=straight, curled=curled)
        card.set_calibration_fields(straight, curled)

    def apply_wizard_value(self, finger: str, phase: str, angle: int):
        self.mapper.set_finger(finger, **{phase: angle})
        cal = self.mapper.get_calibration()[finger]
        self.cards[finger].set_calibration_fields(cal["straight"], cal["curled"])

    def _apply_calibration_to_cards(self):
        for finger, entry in self.mapper.get_calibration().items():
            self.cards[finger].set_calibration_fields(entry["straight"], entry["curled"])

    # ---------------- bottom bar handlers ----------------

    def save_calibration(self):
        self.cfg_mgr.data["calibration"] = self.mapper.get_calibration()
        ok = self.cfg_mgr.save()
        if self.serial.is_connected:
            self.serial.send_calibration(self.mapper.get_calibration())
        self.set_status("Calibration saved" + (" and pushed to device"
                                               if self.serial.is_connected else "")
                        if ok else "Could not write config file — see log")

    def load_calibration(self):
        self.cfg_mgr.load()
        self.mapper.set_calibration(self.cfg_mgr.data["calibration"])
        self._apply_calibration_to_cards()
        if self.serial.is_connected:
            self.serial.send_calibration(self.mapper.get_calibration())
        self.set_status("Calibration loaded from config file")

    def reset_calibration(self):
        self.mapper.reset_defaults()
        self._apply_calibration_to_cards()
        self.set_status("Calibration reset to defaults (not saved yet)")

    def on_home(self):
        if self.serial.send_home():
            self.set_status("Homing hand — driving fingers to the open reference…")
        else:
            self.set_status("Not connected — cannot home")

    def on_test_90(self):
        for card in self.cards.values():
            card.set_slider_silent(90)
        if self.serial.send_angles({f: 90 for f in FINGERS}, force=True):
            self.set_status("All servos commanded to 90°")
        else:
            self.set_status("Not connected — sliders set to 90° only")

    def on_stop(self):
        self.serial.send_stop()
        # Drop to Manual so Track mode doesn't immediately re-drive the servos.
        self.mode_selector.set("Manual")
        self._apply_mode("Manual")
        self.set_status("EMERGENCY STOP — servos detached (switched to Manual mode)")

    def on_quit(self):
        log.info("Shutting down…")
        self.worker.stop()
        if self.serial.is_connected:
            self.serial.send_stop()
        self.serial.close()
        self.destroy()

    # ---------------- misc ----------------

    def open_wizard(self):
        CalibrationWizard(self)

    def _maybe_first_run(self):
        if not self.cfg_mgr.data.get("first_run"):
            return
        self.cfg_mgr.data["first_run"] = False
        self.cfg_mgr.save()
        if messagebox.askyesno(
                "First run",
                "Welcome! Servo calibration maps each finger's curl to your "
                "hand hardware.\n\nRun the calibration wizard now?"):
            self.open_wizard()
