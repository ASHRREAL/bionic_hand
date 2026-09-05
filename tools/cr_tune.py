#!/usr/bin/env python3
"""Interactive calibrator for the continuous-rotation (CR) firmware.

Open-loop CR control needs three numbers per finger:
  * neutral  — the exact command that makes the servo STOP (no creep)
  * sign     — which way it spins so a rising target curls the finger
  * rate     — how fast it travels, so the position estimate tracks reality

This walks you through each, watching the servo, and saves the values into the
ESP32 (they persist in NVS). Run it with all servos wired and powered.

    python tools/cr_tune.py            # tune every finger
    python tools/cr_tune.py thumb ring # tune only these

Firmware commands used: J (raw jog), N/R/G/D (tuning), H (home), S (target).
"""
import glob
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed — run ./setup.sh or 'pip install pyserial'")

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
DRIVE_OFFSET = 40  # must match the firmware default unless you changed it with D


def find_port():
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    # Drop obvious non-ESP devices if we can't tell; just take the last ttyUSB.
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    return (usb or ports or [None])[-1]


class Link:
    def __init__(self, port):
        s = serial.Serial()
        s.port = port
        s.baudrate = 115200
        s.timeout = 0.4
        s.dtr = False
        s.rts = False
        s.open()
        time.sleep(1.8)
        s.reset_input_buffer()
        self.s = s

    def send(self, cmd, settle=0.15):
        self.s.write((cmd + "\n").encode())
        time.sleep(settle)
        out = []
        t0 = time.time()
        while time.time() - t0 < 0.4:
            line = self.s.readline().decode("ascii", "replace").strip()
            if line:
                out.append(line)
        return out

    def ping(self):
        return "PONG" in self.send("P")

    def close(self):
        try:
            self.s.write(b"X\n")
            self.s.close()
        except Exception:
            pass


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        raise SystemExit("aborted")


def tune_neutral(link, finger):
    print(f"\n[{finger}] STEP 1/3 — find the STOP point (neutral)")
    print("  The servo is now driven at its neutral command. If it creeps,")
    print("  nudge the value until it sits perfectly still.")
    neutral = 90
    link.send(f"J,{finger}:{neutral}")
    while True:
        cmd = ask(f"  neutral={neutral}  [+]=up [-]=down [number]=set [ok]=done > ")
        if cmd in ("ok", "o", ""):
            break
        elif cmd in ("+", "="):
            neutral = min(180, neutral + 1)
        elif cmd == "++":
            neutral = min(180, neutral + 5)
        elif cmd == "-":
            neutral = max(0, neutral - 1)
        elif cmd == "--":
            neutral = max(0, neutral - 5)
        elif cmd.lstrip("-").isdigit():
            neutral = max(0, min(180, int(cmd)))
        else:
            print("    (use +, -, ++, --, a number, or 'ok')")
            continue
        link.send(f"J,{finger}:{neutral}")
    link.send(f"N,{finger}:{neutral}")
    link.send(f"J,{finger}:off")
    print(f"  saved neutral={neutral}")
    return neutral


def tune_direction(link, finger):
    print(f"\n[{finger}] STEP 2/3 — spin direction")
    print("  It will spin briefly. Note whether the finger CURLS (closes) or opens.")
    ask("  press Enter to spin > ")
    link.send(f"G,{finger}:1")
    link.send(f"J,{finger}:{90 + DRIVE_OFFSET}")  # sign is applied by G at S-time; raw here shows physical dir
    time.sleep(1.0)
    link.send(f"J,{finger}:off")
    ans = ask("  did the finger CURL/close? [y/n] > ")
    sign = 1 if ans.startswith("y") else -1
    link.send(f"G,{finger}:{sign}")
    print(f"  saved sign={sign:+d}")
    return sign


def tune_rate(link, finger, sign):
    print(f"\n[{finger}] STEP 3/3 — travel speed")
    print("  Homing to the OPEN position, then driving CLOSED. Press Enter the")
    print("  INSTANT it reaches full-closed (or the end of the finger's travel).")
    link.send(f"H,{finger}", settle=2.0)
    ask("  press Enter to START driving closed > ")
    # Drive in the +target (curl) direction using the calibrated sign.
    link.send(f"J,{finger}:{90 + sign * DRIVE_OFFSET}")
    t0 = time.time()
    ask("  ...press Enter at FULL CLOSED > ")
    travel_s = max(0.2, time.time() - t0)
    link.send(f"J,{finger}:off")
    rate = int(round(180.0 / travel_s))
    rate = max(5, min(1000, rate))
    link.send(f"R,{finger}:{rate}")
    print(f"  full travel took {travel_s:.2f}s -> rate={rate} deg/s (saved)")
    return rate


def verify(link, finger):
    print(f"\n[{finger}] check — homing then moving to a few positions; it should")
    print("  spin briefly and STOP at each (roughly: open, half, closed).")
    link.send(f"H,{finger}", settle=2.0)
    for tgt in (30, 120, 75):
        print(f"    target {tgt} ...")
        link.send(f"S,{finger}:{tgt}", settle=2.5)
    link.send(f"J,{finger}:off")


def main():
    args = [a.lower() for a in sys.argv[1:]]
    fingers = [f for f in args if f in FINGERS] or FINGERS
    port = find_port()
    if not port:
        sys.exit("No serial port found (looked for /dev/ttyUSB*). Plug in the ESP32.")
    print(f"Connecting to {port} ...")
    link = Link(port)
    if not link.ping():
        link.close()
        sys.exit("No PONG — is the CR firmware flashed and the cable good?")
    print("Connected. Make sure servos are powered (external 5V, common ground).")
    print(f"Tuning: {', '.join(fingers)}")
    try:
        for finger in fingers:
            print("\n" + "=" * 52)
            tune_neutral(link, finger)
            sign = tune_direction(link, finger)
            tune_rate(link, finger, sign)
            verify(link, finger)
            print(f"[{finger}] done.")
        print("\nAll selected fingers tuned. Values are saved on the ESP32.")
        print("Launch the app with:  .venv/bin/python run.py")
    finally:
        link.close()


if __name__ == "__main__":
    main()
