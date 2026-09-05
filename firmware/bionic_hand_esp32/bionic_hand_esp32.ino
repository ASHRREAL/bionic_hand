/*
 * Bionic hand servo controller — ESP32 (Arduino framework).
 *
 * Drives 5 standard positional servos (50 Hz, 500–2500 µs pulses) from
 * newline-delimited serial commands at 115200 baud:
 *
 *   S,thumb:90,index:45,...          set target angles (degrees, 0–180)
 *   P                                reply "PONG" (host heartbeat)
 *   X                                emergency stop: detach all servos
 *   C,thumb:20,160,index:10,170,...  store calibration bounds (straight,curled)
 *
 * Calibration bounds persist in NVS (Preferences) and every S command is
 * clamped to them, so a buggy host can never drive a servo past the
 * calibrated range. Serial parsing is non-blocking and line-atomic: bytes
 * accumulate in a buffer and a command runs only when its '\n' arrives, so
 * loop() never stalls and a P is answered well inside 10 ms.
 *
 * Libraries: ESP32Servo (install via Library Manager or arduino-cli).
 * Default SERVO_PINS below target an ESP32-C3 Dev Module.
 * Adjust for other boards — on C3 avoid GPIO 12-17 (internal flash),
 * 2/8/9 (strapping), 18/19 (native USB), 20/21 (UART0 console). On the
 * classic WROOM-32 avoid 6-11 (flash), 34-39 (input-only), 0/2/5/12/15
 * (strapping). On S3 avoid 0/3/45/46.
 *
 * POWER: servos must be powered from an external 5V supply able to source
 * several amps, with its ground common to the ESP32 ground. Never power
 * servos from the ESP32's 3V3/5V USB rail.
 */

#include <ESP32Servo.h>
#include <Preferences.h>

static const uint8_t NUM_SERVOS = 5;
static const char *FINGER_NAMES[NUM_SERVOS] = {"thumb", "index", "middle", "ring", "pinky"};
// ESP32-C3 safe PWM output pins: avoids flash (12-17), strapping (2, 8, 9),
// native USB (18, 19), and UART0 console (20, 21).
static const uint8_t SERVO_PINS[NUM_SERVOS] = {3, 10, 5, 6, 7};

static const uint32_t BAUD = 115200;
static const int PULSE_MIN_US = 500;
static const int PULSE_MAX_US = 2500;

// Detach servos if the host goes silent for this long (the host pings every
// 2 s, so a dead host trips this and the hand relaxes). 0 disables.
static const uint32_t FAILSAFE_TIMEOUT_MS = 10000;

Servo servos[NUM_SERVOS];
Preferences prefs;
int calLow[NUM_SERVOS];   // enforced lower bound = min(straight, curled)
int calHigh[NUM_SERVOS];  // enforced upper bound = max(straight, curled)
bool attached[NUM_SERVOS] = {false};
uint32_t lastCommandMs = 0;

char lineBuf[192];
size_t lineLen = 0;
bool lineOverflow = false;

int fingerIndex(const char *name) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (strcmp(name, FINGER_NAMES[i]) == 0) return i;
  }
  return -1;
}

void attachServo(uint8_t i) {
  if (!attached[i]) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], PULSE_MIN_US, PULSE_MAX_US);
    attached[i] = true;
  }
}

void detachAll() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (attached[i]) {
      servos[i].detach();
      attached[i] = false;
    }
  }
}

bool anyAttached() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (attached[i]) return true;
  }
  return false;
}

void loadCalibration() {
  char key[8];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    snprintf(key, sizeof(key), "lo%u", i);
    calLow[i] = prefs.getInt(key, 0);
    snprintf(key, sizeof(key), "hi%u", i);
    calHigh[i] = prefs.getInt(key, 180);
  }
}

void saveCalibration() {
  char key[8];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    snprintf(key, sizeof(key), "lo%u", i);
    prefs.putInt(key, calLow[i]);
    snprintf(key, sizeof(key), "hi%u", i);
    prefs.putInt(key, calHigh[i]);
  }
}

// args: "thumb:90,index:45,..." — move each named servo, clamped to bounds.
void handleSet(char *args) {
  char *save;
  for (char *tok = strtok_r(args, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
    char *colon = strchr(tok, ':');
    if (!colon) continue;
    *colon = '\0';
    int idx = fingerIndex(tok);
    if (idx < 0) continue;
    int angle = atoi(colon + 1);
    angle = constrain(angle, calLow[idx], calHigh[idx]);
    attachServo(idx);
    servos[idx].write(angle);
  }
}

// args: "thumb:20,160,index:10,170,..." — pairs of straight,curled per finger.
void handleCal(char *args) {
  char *save;
  char *tok = strtok_r(args, ",", &save);
  while (tok) {
    char *colon = strchr(tok, ':');
    char *second = strtok_r(NULL, ",", &save);
    if (colon && second) {
      *colon = '\0';
      int idx = fingerIndex(tok);
      if (idx >= 0) {
        int a = constrain(atoi(colon + 1), 0, 180);
        int b = constrain(atoi(second), 0, 180);
        calLow[idx] = min(a, b);
        calHigh[idx] = max(a, b);
      }
    }
    tok = strtok_r(NULL, ",", &save);
  }
  saveCalibration();
  Serial.println("CALOK");
}

void processLine(char *line) {
  lastCommandMs = millis();
  if (line[0] == 'P' && line[1] == '\0') {
    Serial.println("PONG");
  } else if (line[0] == 'X' && line[1] == '\0') {
    detachAll();
    Serial.println("STOPPED");
  } else if (line[0] == 'S' && line[1] == ',') {
    handleSet(line + 2);
  } else if (line[0] == 'C' && line[1] == ',') {
    handleCal(line + 2);
  }
  // Unknown commands are ignored silently.
}

void setup() {
  Serial.begin(BAUD);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  prefs.begin("bionic", false);
  loadCalibration();
  lastCommandMs = millis();
  // Servos stay detached (limp) until the first S command — the hand must
  // not jerk to a position at power-on.
  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineOverflow) {
        lineOverflow = false;  // discard the mangled line
        lineLen = 0;
      } else if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        processLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;  // too long: drop everything until the next newline
    }
  }

  if (FAILSAFE_TIMEOUT_MS > 0 && anyAttached() &&
      millis() - lastCommandMs > FAILSAFE_TIMEOUT_MS) {
    detachAll();
    Serial.println("FAILSAFE");
  }
}
