#!/usr/bin/env bash
# Bionic hand controller — one-shot environment setup.
# Creates a venv, installs Python deps, downloads the MediaPipe hand model,
# and (if arduino-cli is present) installs the ESP32 board package + library.
set -euo pipefail
cd "$(dirname "$0")"

MODEL_URL="https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
ESP32_PKG_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"

echo "==> Checking Python…"
if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 not found. Install Python 3.10 or newer." >&2
    exit 1
fi
if ! python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)'; then
    echo "ERROR: Python 3.10+ required, found $(python3 --version)." >&2
    exit 1
fi
echo "    $(python3 --version) OK"

echo "==> Creating virtual environment (.venv)…"
if [ ! -d .venv ]; then
    python3 -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate

echo "==> Installing Python dependencies…"
pip install --upgrade pip >/dev/null
pip install -r requirements.txt

echo "==> Downloading MediaPipe hand landmark model…"
mkdir -p models
if [ -s models/hand_landmarker.task ]; then
    echo "    models/hand_landmarker.task already present, skipping"
else
    curl -L --fail -o models/hand_landmarker.task "$MODEL_URL"
fi

echo "==> Arduino / ESP32 toolchain…"
if command -v arduino-cli >/dev/null 2>&1; then
    arduino-cli core update-index --additional-urls "$ESP32_PKG_URL"
    arduino-cli core install esp32:esp32 --additional-urls "$ESP32_PKG_URL"
    arduino-cli lib install "ESP32Servo"
    echo "    ESP32 core and ESP32Servo library installed."
    echo "    Flash with, e.g.:"
    echo "      arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/bionic_hand_esp32"
    echo "      arduino-cli upload  --fqbn esp32:esp32:esp32s3 -p /dev/ttyUSB0 firmware/bionic_hand_esp32"
else
    echo "    arduino-cli not found — skipping board package install."
    echo "    Either install arduino-cli, or use the Arduino IDE:"
    echo "      1. Boards Manager URL: $ESP32_PKG_URL"
    echo "      2. Install 'esp32' boards + 'ESP32Servo' library"
    echo "      3. Open firmware/bionic_hand_esp32/bionic_hand_esp32.ino and upload"
fi

echo
echo "Setup complete. Run the app with:"
echo "    .venv/bin/python run.py"
