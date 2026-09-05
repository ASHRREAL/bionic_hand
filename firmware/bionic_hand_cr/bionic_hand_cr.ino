/*
 * Bionic hand — CONTINUOUS-ROTATION servo variant (open-loop position control).
 *
 * Use this sketch ONLY if your servos are continuous-rotation (they spin
 * nonstop instead of holding an angle). For standard positional servos use
 * ../bionic_hand_esp32 instead.
 *
 * A continuous-rotation (CR) servo maps the signal to SPEED, not position:
 *   ~neutral (≈90)  -> stopped
 *   below neutral   -> spin one way   (faster further from neutral)
 *   above neutral   -> spin other way
 * so it has no innate sense of position. This firmware fakes position control
 * open-loop: it holds an ESTIMATED position per finger, drives the motor toward
 * the target, integrates the estimate over elapsed time at a calibrated speed,
 * and commands neutral (stop) once within a deadband of the target. There is no
 * feedback, so the estimate DRIFTS and must be re-homed against a mechanical
 * stop (finger fully open). This is inherently less reliable than a positional
 * servo — see README.
 *
 * Serial protocol (115200 baud, newline-delimited) — superset of the positional
 * firmware so the host app works unchanged, plus CR tuning/homing commands:
 *   S,thumb:120,index:40,...   set target POSITIONS (0-180 virtual), clamped
 *   P                          -> PONG (heartbeat)
 *   X                          emergency stop: detach all (servos coast to stop)
 *   C,thumb:0,180,...          target clamp bounds (min,max) per finger
 *   H                          home: drive all toward "open" stop, set est=0
 *   H,thumb                    home a single finger
 *   N,thumb:90,...             trim neutral (stop) command per finger
 *   R,thumb:120,...            set travel rate (deg/s of estimate) per finger
 *   G,thumb:-1                 set drive direction sign (+1/-1) per finger
 *   D,40                       set global drive offset (distance from neutral)
 *   J,thumb:130                raw jog: write angle directly (suspends control
 *                              on that finger until the next S) — for tuning
 *   J,thumb:off                stop raw jog (writes neutral, stays manual)
 * Replies: READY, PONG, STOPPED, HOMED, CALOK, TUNEOK, JOG.
 * Tuning + clamp bounds persist in NVS.
 *
 * POWER: external 5V supply able to source several amps, ground common with the
 * ESP32. Never power servos from the ESP32 USB rail.
 */

#include <ESP32Servo.h>
#include <Preferences.h>

static const uint8_t NUM_SERVOS = 5;
static const char *FINGER_NAMES[NUM_SERVOS] = {"thumb", "index", "middle", "ring", "pinky"};
static const uint8_t SERVO_PINS[NUM_SERVOS] = {3, 4, 5, 6, 7};  // ESP32-C3 safe pins

static const uint32_t BAUD = 115200;
static const int PULSE_MIN_US = 500;
static const int PULSE_MAX_US = 2500;
static const uint32_t FAILSAFE_TIMEOUT_MS = 10000;

// Per-finger tunables (persisted in NVS, override via serial).
int   neutralAngle[NUM_SERVOS];   // command that stops the CR servo (~90)
int   travelRate[NUM_SERVOS];     // deg/s the estimate advances while driving
int   driveSign[NUM_SERVOS];      // +1/-1: physical direction for a rising target
int   calLow[NUM_SERVOS];         // target clamp lower bound
int   calHigh[NUM_SERVOS];        // target clamp upper bound

// Global tunables.
int      driveOffset = 40;        // how far from neutral while moving -> speed
int      deadband = 3;            // stop when estimate within this of target
uint32_t homingMs = 1500;         // how long to drive toward the open stop

// Runtime state.
float    estPos[NUM_SERVOS];      // estimated position, 0..180 virtual degrees
int      targetAngle[NUM_SERVOS]; // setpoint, 0..180
bool     manual[NUM_SERVOS];      // raw-jog suspends the control loop for a finger
bool     attached[NUM_SERVOS];

Servo servos[NUM_SERVOS];
Preferences prefs;
uint32_t lastCommandMs = 0;
uint32_t lastUpdateMs = 0;

char lineBuf[192];
size_t lineLen = 0;
bool lineOverflow = false;

int fingerIndex(const char *name) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++)
    if (strcmp(name, FINGER_NAMES[i]) == 0) return i;
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
    if (attached[i]) { servos[i].detach(); attached[i] = false; }
    manual[i] = false;
  }
}

bool anyAttached() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) if (attached[i]) return true;
  return false;
}

// ---------------- persistence ----------------

void loadTuning() {
  char key[10];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    snprintf(key, sizeof(key), "n%u", i); neutralAngle[i] = prefs.getInt(key, 90);
    snprintf(key, sizeof(key), "r%u", i); travelRate[i]   = prefs.getInt(key, 120);
    snprintf(key, sizeof(key), "g%u", i); driveSign[i]    = prefs.getInt(key, 1);
    snprintf(key, sizeof(key), "lo%u", i); calLow[i]      = prefs.getInt(key, 0);
    snprintf(key, sizeof(key), "hi%u", i); calHigh[i]     = prefs.getInt(key, 180);
  }
  driveOffset = prefs.getInt("doff", 40);
}

void saveTuning() {
  char key[10];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    snprintf(key, sizeof(key), "n%u", i); prefs.putInt(key, neutralAngle[i]);
    snprintf(key, sizeof(key), "r%u", i); prefs.putInt(key, travelRate[i]);
    snprintf(key, sizeof(key), "g%u", i); prefs.putInt(key, driveSign[i]);
    snprintf(key, sizeof(key), "lo%u", i); prefs.putInt(key, calLow[i]);
    snprintf(key, sizeof(key), "hi%u", i); prefs.putInt(key, calHigh[i]);
  }
  prefs.putInt("doff", driveOffset);
}

// ---------------- control loop ----------------

// Command that spins finger i toward higher (dir=+1) or lower (dir=-1) position.
int driveCommand(uint8_t i, int dir) {
  return constrain(neutralAngle[i] + driveSign[i] * dir * driveOffset, 0, 180);
}

void updateControl() {
  uint32_t now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  lastUpdateMs = now;
  if (dt <= 0.0f || dt > 0.5f) return;  // skip first tick / long stalls

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (manual[i] || !attached[i]) continue;
    float err = targetAngle[i] - estPos[i];
    if (fabs(err) <= deadband) {
      servos[i].write(neutralAngle[i]);          // reached target -> stop/hold
    } else {
      int dir = (err > 0) ? 1 : -1;
      servos[i].write(driveCommand(i, dir));     // drive toward target
      estPos[i] += dir * (float)travelRate[i] * dt;
      estPos[i] = constrain(estPos[i], 0.0f, 180.0f);
    }
  }
}

// ---------------- command handlers ----------------

// Apply f(idx, value) to each "finger:int" token in args.
void forEachPair(char *args, void (*fn)(int, int)) {
  char *save;
  for (char *tok = strtok_r(args, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
    char *colon = strchr(tok, ':');
    if (!colon) continue;
    *colon = '\0';
    int idx = fingerIndex(tok);
    if (idx >= 0) fn(idx, atoi(colon + 1));
  }
}

void setTarget(int idx, int angle) {
  targetAngle[idx] = constrain(angle, calLow[idx], calHigh[idx]);
  attachServo(idx);
  manual[idx] = false;
}
void setNeutral(int idx, int v) { neutralAngle[idx] = constrain(v, 0, 180); }
void setRate(int idx, int v)    { travelRate[idx] = constrain(v, 1, 1000); }
void setSign(int idx, int v)    { driveSign[idx] = (v < 0) ? -1 : 1; }

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
  saveTuning();
  Serial.println("CALOK");
}

// Drive one finger toward the "open" (position 0) mechanical stop, then zero
// the estimate. Homing several fingers runs them in parallel to stay well
// inside the host's heartbeat window.
void homeFingers(bool mask[NUM_SERVOS]) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++)
    if (mask[i]) { attachServo(i); manual[i] = false; }
  uint32_t t0 = millis();
  while (millis() - t0 < homingMs) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++)
      if (mask[i]) servos[i].write(driveCommand(i, -1));
    delay(10);
  }
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (mask[i]) {
      servos[i].write(neutralAngle[i]);
      estPos[i] = 0.0f;
      targetAngle[i] = 0;
    }
  }
  lastUpdateMs = millis();
  Serial.println("HOMED");
}

void handleHome(char *args) {
  bool mask[NUM_SERVOS];
  if (args && *args) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) mask[i] = false;
    int idx = fingerIndex(args);
    if (idx >= 0) mask[idx] = true; else return;
  } else {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) mask[i] = true;
  }
  homeFingers(mask);
}

// J,thumb:130 -> raw write for tuning; J,thumb:off -> write neutral (stay manual)
void handleJog(char *args) {
  char *colon = strchr(args, ':');
  if (!colon) return;
  *colon = '\0';
  int idx = fingerIndex(args);
  if (idx < 0) return;
  attachServo(idx);
  manual[idx] = true;
  const char *val = colon + 1;
  int angle = (strcmp(val, "off") == 0) ? neutralAngle[idx] : constrain(atoi(val), 0, 180);
  servos[idx].write(angle);
  Serial.println("JOG");
}

void processLine(char *line) {
  lastCommandMs = millis();
  if (line[0] == 'P' && line[1] == '\0') {
    Serial.println("PONG");
  } else if (line[0] == 'X' && line[1] == '\0') {
    detachAll();
    Serial.println("STOPPED");
  } else if (line[0] == 'S' && line[1] == ',') {
    forEachPair(line + 2, setTarget);
  } else if (line[0] == 'C' && line[1] == ',') {
    handleCal(line + 2);
  } else if (line[0] == 'H') {
    handleHome(line[1] == ',' ? line + 2 : (char *)"");
  } else if (line[0] == 'N' && line[1] == ',') {
    forEachPair(line + 2, setNeutral); saveTuning(); Serial.println("TUNEOK");
  } else if (line[0] == 'R' && line[1] == ',') {
    forEachPair(line + 2, setRate); saveTuning(); Serial.println("TUNEOK");
  } else if (line[0] == 'G' && line[1] == ',') {
    forEachPair(line + 2, setSign); saveTuning(); Serial.println("TUNEOK");
  } else if (line[0] == 'D' && line[1] == ',') {
    driveOffset = constrain(atoi(line + 2), 0, 90); saveTuning(); Serial.println("TUNEOK");
  } else if (line[0] == 'J' && line[1] == ',') {
    handleJog(line + 2);
  }
  // Unknown commands ignored.
}

void setup() {
  Serial.begin(BAUD);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  prefs.begin("bioniccr", false);
  loadTuning();
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    estPos[i] = 90.0f;        // unknown until homed; matches target -> no motion
    targetAngle[i] = 90;
    manual[i] = false;
    attached[i] = false;
  }
  lastCommandMs = millis();
  lastUpdateMs = millis();
  // Servos stay detached (stopped) until the first command.
  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineOverflow) { lineOverflow = false; lineLen = 0; }
      else if (lineLen > 0) { lineBuf[lineLen] = '\0'; processLine(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }

  updateControl();

  if (FAILSAFE_TIMEOUT_MS > 0 && anyAttached() &&
      millis() - lastCommandMs > FAILSAFE_TIMEOUT_MS) {
    detachAll();
    Serial.println("FAILSAFE");
  }
}
