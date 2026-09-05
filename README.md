# Bionic Hand Controller

Real-time prosthetic hand mirroring: a webcam tracks your hand with MediaPipe,
per-finger curl values (0.0 straight → 1.0 clenched) are mapped through a
calibration table to servo angles, and the angles stream over USB serial to an
ESP32 driving five standard positional servos.

```
webcam ──► MediaPipe Hands ──► curl per finger ──► calibration map ──► serial ──► ESP32 ──► 5 servos
           (21 landmarks)      (smoothed, 0–1)     (straight/curled)   115200 8N1   50 Hz PWM
```

## Hardware

- ESP32 dev board (ESP32-S3 or ESP32-C3 tested pin maps; any ESP32 works)
- 5 × standard positional hobby servos (e.g. MG996R / SG90 class) — **not**
  continuous-rotation servos
- External 5 V supply for the servos (several amps), **ground common with the
  ESP32**. Never power servos from the ESP32's USB rail.

| Finger | GPIO (default, WROOM-32) |
|--------|--------------------------|
| Thumb  | 13 |
| Index  | 14 |
| Middle | 25 |
| Ring   | 26 |
| Pinky  | 27 |

Change `SERVO_PINS` in
[bionic_hand_esp32.ino](firmware/bionic_hand_esp32/bionic_hand_esp32.ino) to
match your wiring. On the classic ESP32-WROOM-32 avoid GPIO 6–11 (internal
flash), 34–39 (input-only, no PWM), and strapping pins 0/2/5/12/15. On S3
avoid 0/3/45/46; on C3 avoid 2/8/9.

## Quick start

```bash
./setup.sh                 # venv + pip deps + model download + ESP32 toolchain
.venv/bin/python run.py    # launch the GUI (-v for debug logging)
```

Manual equivalent:

```bash
pip install -r requirements.txt
mkdir -p models && curl -L -o models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
python run.py
```

### Run on another machine (e.g. your laptop)

```bash
git clone <your-repo-url>
cd bionic_hand
./setup.sh                 # venv + deps + hand model + ESP32 toolchain
.venv/bin/python run.py    # launch the GUI
```

Everything is relative to the repo, so it works from any folder you clone
into. The **webcam is picked up automatically**: `tracking.camera_index`
defaults to `0`, which is your built-in laptop camera. If you plug in a USB
camera instead, set `tracking.camera_index` to `1` or `2` in `config.json`
(the file is created from defaults on the first run; it is never committed).
No local webcam at all? Set `tracking.camera_url` to an Android IP Webcam
stream URL and the app streams from your phone instead.

## Flash the firmware

With arduino-cli (installed by `setup.sh` if present):

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/bionic_hand_esp32
arduino-cli upload  --fqbn esp32:esp32:esp32s3 -p /dev/ttyACM0 firmware/bionic_hand_esp32
```

Or open the `.ino` in the Arduino IDE (install the **esp32** board package and
the **ESP32Servo** library first).

## Using the app

1. **Connect** — pick the ESP32's port in the dropdown (⟳ rescans) and press
   Connect. The dot turns green when the device answers heartbeats; yellow
   means connecting/reconnecting (exponential backoff, automatic).
2. **Calibrate** — first run offers the wizard: for each finger it jogs the
   servo while you confirm the *straight* then *curled* position. You can also
   calibrate by hand: switch to **Calibrate** mode, use the ±1° jog buttons or
   slider, then press *Set Straight* / *Set Curled* on each card. **Save
   Calibration** writes `config.json` and pushes the bounds to the ESP32,
   which clamps every future command to them.
3. **Track** — show one hand to the camera. Press **Reset Baseline** while
   holding your hand relaxed so your natural resting curl reads as zero.
   Toggle **Show Debug** to overlay curl values and bars on the video.
4. **Manual** mode drives each servo directly from its slider —
   useful for bench testing, as is **Test 90°** (centers all servos).
5. **STOP** sends an emergency stop: the ESP32 detaches all servos (no PWM,
   hand goes limp) and the app drops to Manual mode so tracking doesn't
   instantly re-drive them.

## Continuous-rotation servos (open-loop mode)

Standard positional servos are strongly preferred. If your servos are
**continuous-rotation** (they spin nonstop instead of holding an angle — test:
sweep the angle slowly; a CR servo has one neutral point where it stops and
reverses), use the alternate firmware and mode:

1. Flash [firmware/bionic_hand_cr](firmware/bionic_hand_cr) instead of
   `bionic_hand_esp32`:
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32c3 firmware/bionic_hand_cr
   arduino-cli upload --fqbn "esp32:esp32:esp32c3:FlashMode=dio" -p /dev/ttyUSB0 firmware/bionic_hand_cr
   ```
2. Set `servo.continuous_rotation: true` in `config.json` (the app then homes
   the hand on connect and shows a **Home** button).
3. Calibrate each finger's stop-point, direction, and speed:
   ```bash
   .venv/bin/python tools/cr_tune.py         # all fingers, interactive
   ```

**How it works and its limits:** a CR servo maps the signal to *speed*, not
position, so the firmware fakes position control open-loop — it estimates each
finger's position, drives toward the target, and stops within a deadband. There
is **no feedback**, so the estimate drifts and must be re-homed against the
finger's fully-open mechanical stop (the **Home** button, or automatically on
connect). Expect less precision and occasional drift versus positional servos.
CR-only serial commands (`H` home, `N`/`R`/`G`/`D` tuning, `J` raw jog) are
documented in the firmware header.

## Serial protocol (115200 baud, newline-delimited)

| Command | Direction | Meaning |
|---------|-----------|---------|
| `S,thumb:90,index:45,middle:80,ring:75,pinky:60` | host → ESP32 | set target angles (any subset of fingers) |
| `P` | host → ESP32 | heartbeat ping |
| `PONG` | ESP32 → host | ping reply (host expects one within 6 s) |
| `X` | host → ESP32 | emergency stop — detach all servos |
| `C,thumb:20,160,index:10,170,…` | host → ESP32 | calibration bounds (straight,curled) per finger; persisted to NVS |
| `CALOK` / `STOPPED` / `READY` / `FAILSAFE` | ESP32 → host | acknowledgements (informational) |

The host only transmits `S` when at least one servo target changed by more
than `serial.change_threshold_deg` (default 2°), so the link is quiet while
your hand is still. The firmware detaches all servos if it hears nothing for
10 s (host crashed/unplugged) — the heartbeat keeps it alive during normal
idle.

## Configuration (`config.json`)

Created with defaults on first run, next to `run.py`:

| Key | Default | Meaning |
|-----|---------|---------|
| `serial.port` / `serial.baud` | `""` / `115200` | last used port, baud rate |
| `serial.change_threshold_deg` | `2.0` | min angle change before sending |
| `tracking.model_path` | `models/hand_landmarker.task` | MediaPipe model file |
| `tracking.camera_index` | `0` | OpenCV camera index |
| `tracking.smoothing` | `0.35` | EMA weight of the newest curl sample (higher = snappier, lower = smoother) |
| `tracking.min_detection_confidence` | `0.6` | MediaPipe detection threshold |
| `servo.dead_zone` | `0.06` | curl band at each extreme snapped to 0/1 |
| `calibration.<finger>` | `{straight: 10, curled: 170}` | per-finger servo bounds |
| `debug_overlay` | `true` | debug overlay on by default |

## Architecture

- [app/hand_tracker.py](app/hand_tracker.py) — MediaPipe HandLandmarker (VIDEO
  mode); curl from the mean of two joint angles per finger, exponentially
  smoothed, baseline-offset.
- [app/servo_mapper.py](app/servo_mapper.py) — curl → angle linear
  interpolation with endpoint dead zone; lock-protected calibration table.
- [app/serial_link.py](app/serial_link.py) — manager thread owns the port:
  heartbeat, PONG watchdog, auto-reconnect with exponential backoff; all
  writes serialized through an RLock.
- [app/capture_worker.py](app/capture_worker.py) — daemon capture thread:
  camera → inference → curls → angles → serial; publishes annotated frames
  into a lock-protected snapshot.
- [app/gui.py](app/gui.py) — Tk thread polls the snapshot with `after()`;
  serial callbacks are marshalled through a queue. No widget is ever touched
  off-thread.

## Troubleshooting

- **"Camera not found"** — another app is holding the webcam, or the index is
  wrong: set `tracking.camera_index` to 1 or 2 in `config.json`.
- **"Hand model unavailable"** — `models/hand_landmarker.task` is missing;
  re-run `./setup.sh` or the curl command above.
- **No ports in the dropdown** — on Linux add yourself to the serial group
  (`sudo usermod -aG uucp $USER` on Arch, `dialout` on Debian) and re-login;
  press ⟳ after plugging in.
- **Servos twitch at rest** — raise `servo.dead_zone` or lower
  `tracking.smoothing`; press Reset Baseline with your hand relaxed.
- **Servos hit mechanical limits** — recalibrate; the ESP32 clamps to the
  calibrated range, so fix the straight/curled bounds rather than the code.
- **Yellow dot forever** — wrong port selected, or the firmware isn't flashed
  (the host needs a PONG within 6 s of pinging).
