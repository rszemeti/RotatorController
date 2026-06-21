// Dual-core rotator firmware
// Core 0: comms, reporting, EEPROM persistence
// Core 1: motion only (FastAccelStepper, drive enable)
// Inter-core: rp2040.fifo for commands (Core0->Core1), volatile shared state (Core1->Core0)
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <EEPROM.h>

// ============================================================
// FIFO command words  (upper 4 bits = opcode, lower 28 = data)
// ============================================================
#define CMD_GOTO        0x10000000UL  // data = target steps (signed, cast)
#define CMD_PAN         0x20000000UL  // data = 1 forward, 0xFF backward
#define CMD_STOP        0x30000000UL
#define CMD_FORCE_STOP  0x40000000UL
#define CMD_SET_SPEED   0x50000000UL  // data = speed in Hz (integer)
#define CMD_SET_ACCEL   0x60000000UL  // data = accel (integer)
#define CMD_SET_POS     0x70000000UL  // data = new step position (signed, cast)
#define CMD_OPCODE(w)   ((w) & 0xF0000000UL)
#define CMD_DATA(w)     ((w) & 0x0FFFFFFFUL)
#define CMD_SDATA(w)    ((int32_t)(((w) & 0x0FFFFFFFUL) << 4) >> 4) // sign-extend 28-bit

// ============================================================
// Shared state — written ONLY by Core 1, read by Core 0
// ============================================================
volatile long  sharedPosition   = 0;   // current stepper position
volatile bool  sharedIsRunning  = false;
volatile bool  sharedIsPanning  = false;

// Motor Configuration
const int STEPS_PER_REV = 200;
const int GEAR_RATIO = 50 * 78/ 13; // 50:1 gearbox with 78/13 final drive reduction
const int MICROSTEPS = 10;
const long DEFAULT_TOTAL_STEPS = (long)STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;

// Pin definitions
const int stepPin = 9;
const int dirPin = 8;
const int driveDisablePin = 10;
const int serialTxPin = 4;
const int serialRxPin = 5;
const unsigned long serialBaudRate = 19200;
const unsigned long usbBaudRate = 115200;

#define ROTATOR_SERIAL Serial2

// EEPROM addresses
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_POSITION_ADDR = 4;
const int EEPROM_SPEED_ADDR = 8;
const int EEPROM_ACCEL_ADDR = 12;
const int EEPROM_STEPS_ADDR = 16;
const int EEPROM_SIZE_BYTES = 512;
const uint16_t EEPROM_MAGIC = 0xAE42;

// Default values
const float DEFAULT_SPEED = 4000.0;
const float DEFAULT_ACCEL = 2000.0;
const float PAN_CONSTANT_SPEED = 3000.0;

// ============================================================
// Core 1 — Motion globals (never touched by Core 0 after boot)
// ============================================================
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Motion parameters — set from Core 0 via FIFO commands
float  c1MaxSpeed  = DEFAULT_SPEED;
float  c1Accel     = DEFAULT_ACCEL;
bool   c1Panning   = false;
int    c1PanDir    = 0;
unsigned long c1LastPanCmd = 0;
bool   c1HasTarget = false;
long   c1TargetSteps = 0;
bool   c1DriveEnabled = false;
unsigned long c1LastMotionMs = 0;
const unsigned long PAN_TIMEOUT         = 1000;
const long          DEADBAND_STEPS      = 20;
const unsigned long driveEnableSettleMs = 500;
const unsigned long driveIdleDisableMs  = 10000;

// ============================================================
// Core 0 — Comms / persistence globals
// ============================================================
float currentMaxSpeed            = DEFAULT_SPEED;
float currentAccel               = DEFAULT_ACCEL;
long  currentStepsPerRevolution  = DEFAULT_TOTAL_STEPS;
float positionAngleOffsetDeg     = 0.0;

unsigned long lastPositionUpdate = 0;
const unsigned long positionUpdateInterval      = 200;
unsigned long lastPositionSave   = 0;
const unsigned long positionSaveInterval        = 5000;
const unsigned long positionIdleForceSaveInterval = 30000;
unsigned long lastMotionForPositionSave = 0;
bool  pendingPositionSave  = false;
long  pendingPositionSteps = 0;

const unsigned long heartbeatIntervalMs = 500;
unsigned long lastHeartbeatToggle = 0;
bool heartbeatState = false;
const int heartbeatFallbackPin = 25;
const unsigned long usbStatusIntervalMs = 1000;
unsigned long lastUsbStatus = 0;

long  lastSavedPosition           = 0;
float lastSavedSpeed              = 0;
float lastSavedAccel              = 0;
long  lastSavedStepsPerRevolution = DEFAULT_TOTAL_STEPS;
const long POSITION_SAVE_THRESHOLD = 50;

// ============================================================
// Helpers shared by both cores (read-only constants / pure math)
// ============================================================
float normalizeAngleDeg(float angle) {
  while (angle < 0)      angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

float getCurrentAngle() {
  // Uses sharedPosition (volatile, written by Core 1)
  float rel = (float)sharedPosition * 360.0f / currentStepsPerRevolution;
  return normalizeAngleDeg(rel + positionAngleOffsetDeg);
}

long calculateNearestTargetSteps(float targetAngle, long referencePos) {
  float relTarget = normalizeAngleDeg(targetAngle - positionAngleOffsetDeg);
  long  base      = (long)round((double)relTarget * (double)currentStepsPerRevolution / 360.0);
  long  turn      = (long)round((double)(referencePos - base) / (double)currentStepsPerRevolution);
  return base + turn * currentStepsPerRevolution;
}

// ============================================================
// Core 0 forward declarations
// ============================================================
void processCommand(String command);
void goToAngle(float targetAngle);
void setCurrentPosition(float angle);
void setSpeed(float speed);
void setAcceleration(float accel);
void setStepsPerRevolution(long steps);
void printPosition();
void printInfo();
void loadFromEEPROM();
void savePositionToEEPROM(long position);
void saveSpeedToEEPROM(float speed);
void saveAccelToEEPROM(float accel);
void saveStepsToEEPROM(long steps);
void commitEEPROM();
void forceSaveAll();
void resetEEPROM();

// ============================================================
// CORE 0 — setup / loop  (comms, reporting, EEPROM)
// ============================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(heartbeatFallbackPin, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(heartbeatFallbackPin, LOW);

  Serial.begin(usbBaudRate);
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart < 2000)) delay(10);
  if (Serial) Serial.println(F("BOOT OK"));

  ROTATOR_SERIAL.setTX(serialTxPin);
  ROTATOR_SERIAL.setRX(serialRxPin);
  ROTATOR_SERIAL.begin(serialBaudRate);

  EEPROM.begin(EEPROM_SIZE_BYTES);
  loadFromEEPROM();   // populates currentMaxSpeed, currentAccel, etc.

  // Wait for Core 1 to signal ready via FIFO
  rp2040.fifo.pop();  // blocks until Core 1 pushes 0xDEAD

  // Send initial parameters to Core 1
  rp2040.fifo.push(CMD_SET_SPEED | (uint32_t)(int32_t)currentMaxSpeed);
  rp2040.fifo.push(CMD_SET_ACCEL | (uint32_t)(int32_t)currentAccel);

  ROTATOR_SERIAL.println(F("=== Rotator Ready (Dual-Core) ==="));
  ROTATOR_SERIAL.print(F("Loaded Steps/360: ")); ROTATOR_SERIAL.println(currentStepsPerRevolution);
  ROTATOR_SERIAL.print(F("Loaded Position: ")); ROTATOR_SERIAL.print(getCurrentAngle(), 2); ROTATOR_SERIAL.println(F("°"));
  ROTATOR_SERIAL.print(F("Loaded Speed: ")); ROTATOR_SERIAL.println(currentMaxSpeed, 0);
  ROTATOR_SERIAL.print(F("Loaded Accel: ")); ROTATOR_SERIAL.println(currentAccel, 0);
  ROTATOR_SERIAL.println(F("GUI Ready"));
  printPosition();
}

void loop() {
  // --- Heartbeat ---
  if (millis() - lastHeartbeatToggle >= heartbeatIntervalMs) {
    heartbeatState = !heartbeatState;
    digitalWrite(LED_BUILTIN,        heartbeatState ? HIGH : LOW);
    digitalWrite(heartbeatFallbackPin, heartbeatState ? HIGH : LOW);
    lastHeartbeatToggle = millis();
  }

  // --- USB keepalive ---
  if (Serial && (millis() - lastUsbStatus >= usbStatusIntervalMs)) {
    Serial.println(F("ALIVE"));
    lastUsbStatus = millis();
  }

  // --- Track motion activity (reads volatile) ---
  bool moving = sharedIsRunning || sharedIsPanning;
  if (moving) lastMotionForPositionSave = millis();

  // --- Position reporting ---
  if (millis() - lastPositionUpdate >= positionUpdateInterval) {
    if (moving) printPosition();
    lastPositionUpdate = millis();
  }

  // --- Periodic position save (deferred until idle) ---
  if (millis() - lastPositionSave >= positionSaveInterval) {
    long pos = sharedPosition;
    if (abs(pos - lastSavedPosition) >= POSITION_SAVE_THRESHOLD) {
      if (moving) {
        pendingPositionSave  = true;
        pendingPositionSteps = pos;
      } else {
        savePositionToEEPROM(pos);
        lastSavedPosition = pos;
      }
    }
    lastPositionSave = millis();
  }

  // --- Flush deferred save once idle ---
  if (pendingPositionSave && !moving) {
    savePositionToEEPROM(pendingPositionSteps);
    lastSavedPosition    = pendingPositionSteps;
    pendingPositionSave  = false;
  }

  // --- Idle force-save fallback ---
  if (!moving && (millis() - lastMotionForPositionSave >= positionIdleForceSaveInterval)) {
    long pos = sharedPosition;
    if (pos != lastSavedPosition) {
      savePositionToEEPROM(pos);
      lastSavedPosition = pos;
    }
  }

  // --- Serial command processing ---
  if (ROTATOR_SERIAL.available() > 0) {
    String command = ROTATOR_SERIAL.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    processCommand(command);
  }
}

// ============================================================
// CORE 1 — setup1 / loop1  (motion only)
// ============================================================
void setup1() {
  pinMode(driveDisablePin, OUTPUT);
  digitalWrite(driveDisablePin, HIGH);  // drive disabled at boot

  engine.init();
  stepper = engine.stepperConnectToPin(stepPin);
  if (stepper) {
    stepper->setDirectionPin(dirPin);
    stepper->setAutoEnable(true);
    stepper->setSpeedInHz((uint32_t)c1MaxSpeed);
    stepper->setAcceleration((uint32_t)c1Accel);
  }

  rp2040.fifo.push(0xDEAD);  // signal Core 0 we are ready
}

void loop1() {
  // --- Process any pending FIFO commands from Core 0 ---
  while (rp2040.fifo.available()) {
    uint32_t cmd = rp2040.fifo.pop();
    uint32_t op  = CMD_OPCODE(cmd);

    if (op == CMD_GOTO) {
      long target = (long)CMD_SDATA(cmd);
      c1HasTarget   = true;
      c1TargetSteps = target;
      c1Panning     = false;
      if (!c1DriveEnabled) {
        digitalWrite(driveDisablePin, LOW);
        c1DriveEnabled = true;
        delay(driveEnableSettleMs);
      }
      c1LastMotionMs = millis();
      stepper->setSpeedInHz((uint32_t)c1MaxSpeed);
      stepper->moveTo(target);

    } else if (op == CMD_PAN) {
      c1HasTarget = false;
      c1PanDir    = (CMD_DATA(cmd) == 1) ? 1 : -1;
      c1Panning   = true;
      c1LastPanCmd = millis();
      if (!c1DriveEnabled) {
        digitalWrite(driveDisablePin, LOW);
        c1DriveEnabled = true;
        delay(driveEnableSettleMs);
      }
      c1LastMotionMs = millis();
      stepper->setSpeedInHz((uint32_t)PAN_CONSTANT_SPEED);
      if (c1PanDir > 0) stepper->runForward();
      else              stepper->runBackward();

    } else if (op == CMD_STOP) {
      c1Panning   = false;
      c1HasTarget = false;
      c1PanDir    = 0;
      stepper->stopMove();
      stepper->setSpeedInHz((uint32_t)c1MaxSpeed);

    } else if (op == CMD_FORCE_STOP) {
      c1Panning   = false;
      c1HasTarget = false;
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());

    } else if (op == CMD_SET_SPEED) {
      c1MaxSpeed = (float)CMD_DATA(cmd);
      if (!c1Panning) stepper->setSpeedInHz((uint32_t)c1MaxSpeed);

    } else if (op == CMD_SET_ACCEL) {
      c1Accel = (float)CMD_DATA(cmd);
      stepper->setAcceleration((uint32_t)c1Accel);

    } else if (op == CMD_SET_POS) {
      stepper->setCurrentPosition((long)CMD_SDATA(cmd));
      c1HasTarget = false;
    }
  }

  // --- Pan watchdog ---
  if (c1Panning && (millis() - c1LastPanCmd > PAN_TIMEOUT)) {
    c1Panning = false;
    c1PanDir  = 0;
    stepper->stopMove();
    stepper->setSpeedInHz((uint32_t)c1MaxSpeed);
  }

  // --- Active target re-trigger (if stepper stopped short) ---
  if (c1HasTarget && !c1Panning && !stepper->isRunning()) {
    long pos = stepper->getCurrentPosition();
    if (abs(c1TargetSteps - pos) <= DEADBAND_STEPS) {
      c1HasTarget = false;
    } else {
      stepper->moveTo(c1TargetSteps);
    }
  }

  // --- Drive idle disable ---
  bool running = stepper->isRunning() || c1Panning;
  if (running) {
    c1LastMotionMs = millis();
  } else if (c1DriveEnabled && (millis() - c1LastMotionMs >= driveIdleDisableMs)) {
    digitalWrite(driveDisablePin, HIGH);
    c1DriveEnabled = false;
  }

  // --- Update shared state for Core 0 to read ---
  sharedPosition  = stepper->getCurrentPosition();
  sharedIsRunning = stepper->isRunning();
  sharedIsPanning = c1Panning;
}

// ============================================================
// CORE 0 — Command processing (pushes to FIFO, no stepper calls)
// ============================================================
void processCommand(String command) {
  if (command.startsWith("AC")) {
    float accel = command.substring(2).toFloat();
    setAcceleration(accel);
  }
  else if (command.startsWith("A")) {
    float angle = command.substring(1).toFloat();
    rp2040.fifo.push(CMD_STOP);
    goToAngle(angle);
  }
  else if (command == "H") {
    rp2040.fifo.push(CMD_STOP);
    goToAngle(0);
  }
  else if (command == "P") {
    printPosition();
  }
  else if (command.startsWith("SETPOS")) {
    rp2040.fifo.push(CMD_STOP);
    float angle = command.substring(6).toFloat();
    setCurrentPosition(angle);
  }
  else if (command == "PANLEFT") {
    // Refresh pan watchdog if already panning left
    if (sharedIsPanning) {
      rp2040.fifo.push(CMD_PAN | 0xFF);  // 0xFF = backward
    } else {
      rp2040.fifo.push(CMD_PAN | 0xFF);
    }
    ROTATOR_SERIAL.println(F("PAN:L"));
  }
  else if (command == "PANRIGHT") {
    rp2040.fifo.push(CMD_PAN | 1);
    ROTATOR_SERIAL.println(F("PAN:R"));
  }
  else if (command == "PANSTOP") {
    rp2040.fifo.push(CMD_STOP);
    ROTATOR_SERIAL.println(F("PANSTOP"));
  }
  else if (command == "STOP") {
    rp2040.fifo.push(CMD_FORCE_STOP);
    long pos = sharedPosition;
    if (pos != lastSavedPosition) {
      savePositionToEEPROM(pos);
      lastSavedPosition = pos;
    }
    ROTATOR_SERIAL.println(F("STOP"));
  }
  else if (command.startsWith("STEPS")) {
    long steps = command.substring(5).toInt();
    setStepsPerRevolution(steps);
  }
  else if (command.startsWith("S") && !command.startsWith("SETPOS") && !command.startsWith("STOP") && !command.startsWith("SAVE")) {
    float speed = command.substring(1).toFloat();
    setSpeed(speed);
  }
  else if (command == "SAVE") {
    forceSaveAll();
  }
  else if (command == "LOAD") {
    loadFromEEPROM();
    rp2040.fifo.push(CMD_SET_SPEED | (uint32_t)(int32_t)currentMaxSpeed);
    rp2040.fifo.push(CMD_SET_ACCEL | (uint32_t)(int32_t)currentAccel);
    ROTATOR_SERIAL.println(F("LOADED"));
    printPosition();
  }
  else if (command == "RESET") {
    resetEEPROM();
  }
  else if (command == "INFO") {
    printInfo();
  }
}

void goToAngle(float targetAngle) {
  while (targetAngle < 0)       targetAngle += 360.0f;
  while (targetAngle >= 360.0f) targetAngle -= 360.0f;

  long currentPos = sharedPosition;
  long target     = calculateNearestTargetSteps(targetAngle, currentPos);
  long delta      = target - currentPos;

  ROTATOR_SERIAL.print(F("GO:")); ROTATOR_SERIAL.println(targetAngle, 1);
  ROTATOR_SERIAL.print(F("GO DBG cur=")); ROTATOR_SERIAL.print(currentPos);
  ROTATOR_SERIAL.print(F(" dStp="));     ROTATOR_SERIAL.print(delta);
  ROTATOR_SERIAL.print(F(" tgt="));      ROTATOR_SERIAL.print(target);
  ROTATOR_SERIAL.print(F(" spr="));      ROTATOR_SERIAL.println(currentStepsPerRevolution);

  if (delta != 0) {
    // Pack signed target into 28-bit payload
    rp2040.fifo.push(CMD_GOTO | ((uint32_t)(int32_t)target & 0x0FFFFFFFUL));
  }
}

void setCurrentPosition(float angle) {
  angle = normalizeAngleDeg(angle);
  long pos = sharedPosition;
  float relAngle = (float)pos * 360.0f / currentStepsPerRevolution;
  positionAngleOffsetDeg = normalizeAngleDeg(angle - relAngle);

  savePositionToEEPROM(pos);
  lastSavedPosition = pos;

  ROTATOR_SERIAL.print(F("SET:")); ROTATOR_SERIAL.println(angle, 1);
}

void setSpeed(float speed) {
  if (speed < 100)   { ROTATOR_SERIAL.println(F("Speed too low, setting to 100"));  speed = 100; }
  if (speed > 50000) { ROTATOR_SERIAL.println(F("Speed too high, limiting to 50000")); speed = 50000; }

  currentMaxSpeed = speed;
  rp2040.fifo.push(CMD_SET_SPEED | (uint32_t)(int32_t)speed);

  if (abs(speed - lastSavedSpeed) > 1.0f) {
    saveSpeedToEEPROM(speed);
    lastSavedSpeed = speed;
  }
  ROTATOR_SERIAL.print(F("SPD:")); ROTATOR_SERIAL.println(speed, 0);
}

void setAcceleration(float accel) {
  if (accel < 100)    { ROTATOR_SERIAL.println(F("Accel too low, setting to 100"));    accel = 100; }
  if (accel > 100000) { ROTATOR_SERIAL.println(F("Accel too high, limiting to 100000")); accel = 100000; }

  currentAccel = accel;
  rp2040.fifo.push(CMD_SET_ACCEL | (uint32_t)(int32_t)accel);

  if (abs(accel - lastSavedAccel) > 1.0f) {
    saveAccelToEEPROM(accel);
    lastSavedAccel = accel;
  }
  ROTATOR_SERIAL.print(F("ACC:")); ROTATOR_SERIAL.println(accel, 0);
}

void setStepsPerRevolution(long steps) {
  if (steps < 1000)     { ROTATOR_SERIAL.println(F("Steps/360 too low, setting to 1000"));    steps = 1000; }
  if (steps > 10000000) { ROTATOR_SERIAL.println(F("Steps/360 too high, limiting to 10000000")); steps = 10000000; }

  if (sharedIsRunning || sharedIsPanning) {
    ROTATOR_SERIAL.println(F("Cannot change Steps/360 while moving"));
    return;
  }
  if (steps == currentStepsPerRevolution) {
    ROTATOR_SERIAL.print(F("STEPS:")); ROTATOR_SERIAL.println(steps);
    return;
  }

  long  pos        = sharedPosition;
  float curAngle   = getCurrentAngle();
  float relAngle   = normalizeAngleDeg(curAngle - positionAngleOffsetDeg);
  long  remapBase  = (long)round((double)relAngle * (double)steps / 360.0);
  long  nearTurn   = (long)round((double)(pos - remapBase) / (double)steps);
  long  remapPos   = remapBase + nearTurn * steps;

  rp2040.fifo.push(CMD_SET_POS | ((uint32_t)(int32_t)remapPos & 0x0FFFFFFFUL));

  currentStepsPerRevolution = steps;
  savePositionToEEPROM(remapPos);
  lastSavedPosition = remapPos;

  if (steps != lastSavedStepsPerRevolution) {
    saveStepsToEEPROM(steps);
    lastSavedStepsPerRevolution = steps;
  }
  ROTATOR_SERIAL.print(F("STEPS:")); ROTATOR_SERIAL.println(steps);
}

void printPosition() {
  ROTATOR_SERIAL.print(F("Position: "));
  ROTATOR_SERIAL.print(getCurrentAngle(), 2);
  ROTATOR_SERIAL.println(F("°"));
}

void printInfo() {
  ROTATOR_SERIAL.println(F("\n=== Settings ==="));
  ROTATOR_SERIAL.print(F("MaxSpeed: "));   ROTATOR_SERIAL.println(currentMaxSpeed, 0);
  ROTATOR_SERIAL.print(F("Accel: "));      ROTATOR_SERIAL.println(currentAccel, 0);
  ROTATOR_SERIAL.print(F("PanSpeed: "));   ROTATOR_SERIAL.println(PAN_CONSTANT_SPEED);
  ROTATOR_SERIAL.print(F("Steps/360: "));  ROTATOR_SERIAL.println(currentStepsPerRevolution);
  ROTATOR_SERIAL.print(F("Panning: "));    ROTATOR_SERIAL.println(sharedIsPanning ? F("YES") : F("NO"));
  ROTATOR_SERIAL.print(F("Running: "));    ROTATOR_SERIAL.println(sharedIsRunning ? F("YES") : F("NO"));
  ROTATOR_SERIAL.print(F("Position delta: "));
  ROTATOR_SERIAL.print(abs(sharedPosition - lastSavedPosition));
  ROTATOR_SERIAL.print(F(" (saves at ")); ROTATOR_SERIAL.print(POSITION_SAVE_THRESHOLD); ROTATOR_SERIAL.println(F(")"));
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  ROTATOR_SERIAL.print(F("EEPROM Magic: 0x")); ROTATOR_SERIAL.println(magic, HEX);
  printPosition();
}

// ========== EEPROM Functions ==========

void loadFromEEPROM() {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  ROTATOR_SERIAL.print(F("EEPROM Magic: 0x"));
  ROTATOR_SERIAL.println(magic, HEX);

  if (magic == EEPROM_MAGIC) {
    // Load speed
    float savedSpeed;
    EEPROM.get(EEPROM_SPEED_ADDR, savedSpeed);
    ROTATOR_SERIAL.print(F("EEPROM Speed: ")); ROTATOR_SERIAL.println(savedSpeed, 0);

    if (savedSpeed >= 100 && savedSpeed <= 50000) {
      currentMaxSpeed = savedSpeed;
      lastSavedSpeed = savedSpeed;
    } else {
      ROTATOR_SERIAL.println(F("Speed out of range, using default"));
      currentMaxSpeed = DEFAULT_SPEED;
      lastSavedSpeed = DEFAULT_SPEED;
      // Fix corrupted speed in EEPROM
      EEPROM.put(EEPROM_SPEED_ADDR, DEFAULT_SPEED);
      commitEEPROM();
    }

    // Load acceleration
    float savedAccel;
    EEPROM.get(EEPROM_ACCEL_ADDR, savedAccel);
    ROTATOR_SERIAL.print(F("EEPROM Accel: ")); ROTATOR_SERIAL.println(savedAccel, 0);

    if (savedAccel >= 100 && savedAccel <= 100000) {
      currentAccel = savedAccel;
      lastSavedAccel = savedAccel;
    } else {
      ROTATOR_SERIAL.println(F("Accel out of range, using default"));
      currentAccel = DEFAULT_ACCEL;
      lastSavedAccel = DEFAULT_ACCEL;
      // Fix corrupted accel in EEPROM
      EEPROM.put(EEPROM_ACCEL_ADDR, DEFAULT_ACCEL);
      commitEEPROM();
    }

    // Load steps per revolution
    long savedSteps;
    EEPROM.get(EEPROM_STEPS_ADDR, savedSteps);
    ROTATOR_SERIAL.print(F("EEPROM Steps/360: ")); ROTATOR_SERIAL.println(savedSteps);

    if (savedSteps >= 1000 && savedSteps <= 10000000) {
      currentStepsPerRevolution = savedSteps;
      lastSavedStepsPerRevolution = savedSteps;
    } else {
      ROTATOR_SERIAL.println(F("Steps/360 out of range, using default"));
      currentStepsPerRevolution = DEFAULT_TOTAL_STEPS;
      lastSavedStepsPerRevolution = DEFAULT_TOTAL_STEPS;
      EEPROM.put(EEPROM_STEPS_ADDR, DEFAULT_TOTAL_STEPS);
      commitEEPROM();
    }

    // Load saved angle reference and map it to runtime step position 0 at boot.
    long savedPosition;
    EEPROM.get(EEPROM_POSITION_ADDR, savedPosition);
    positionAngleOffsetDeg = normalizeAngleDeg((float)savedPosition * 360.0 / currentStepsPerRevolution);
    stepper->setCurrentPosition(0);
    lastSavedPosition = 0;

    ROTATOR_SERIAL.println(F("EEPROM: Loaded"));
  } else {
    ROTATOR_SERIAL.println(F("EEPROM: No valid data, initializing defaults"));
    currentMaxSpeed = DEFAULT_SPEED;
    currentAccel = DEFAULT_ACCEL;
    positionAngleOffsetDeg = 0.0;
    stepper->setCurrentPosition(0);

    lastSavedSpeed = DEFAULT_SPEED;
    lastSavedAccel = DEFAULT_ACCEL;
    currentStepsPerRevolution = DEFAULT_TOTAL_STEPS;
    lastSavedStepsPerRevolution = DEFAULT_TOTAL_STEPS;
    lastSavedPosition = 0;

    // Initialize EEPROM with valid data
    EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    EEPROM.put(EEPROM_POSITION_ADDR, 0L);
    EEPROM.put(EEPROM_SPEED_ADDR, DEFAULT_SPEED);
    EEPROM.put(EEPROM_ACCEL_ADDR, DEFAULT_ACCEL);
    EEPROM.put(EEPROM_STEPS_ADDR, DEFAULT_TOTAL_STEPS);
    commitEEPROM();
    ROTATOR_SERIAL.println(F("EEPROM: Initialized"));
  }
}

void savePositionToEEPROM(long position) {
  (void)position;
  // EEPROM.put only writes if value changed (built-in optimization)
  long storedPosition = (long)round((double)getCurrentAngle() * (double)currentStepsPerRevolution / 360.0);
  EEPROM.put(EEPROM_POSITION_ADDR, storedPosition);
  commitEEPROM();
}

void saveSpeedToEEPROM(float speed) {
  EEPROM.put(EEPROM_SPEED_ADDR, speed);
  commitEEPROM();
}

void saveAccelToEEPROM(float accel) {
  EEPROM.put(EEPROM_ACCEL_ADDR, accel);
  commitEEPROM();
}

void saveStepsToEEPROM(long steps) {
  EEPROM.put(EEPROM_STEPS_ADDR, steps);
  commitEEPROM();
}

void commitEEPROM() {
  EEPROM.commit();
}

void forceSaveAll() {
  long currentPos = sharedPosition;
  long storedPosition = (long)round((double)getCurrentAngle() * (double)currentStepsPerRevolution / 360.0);

  ROTATOR_SERIAL.println(F("Force saving all to EEPROM:"));
  ROTATOR_SERIAL.print(F("  Position: ")); ROTATOR_SERIAL.println(getCurrentAngle(), 2);
  ROTATOR_SERIAL.print(F("  Speed: ")); ROTATOR_SERIAL.println(currentMaxSpeed, 0);
  ROTATOR_SERIAL.print(F("  Accel: ")); ROTATOR_SERIAL.println(currentAccel, 0);
  ROTATOR_SERIAL.print(F("  Steps/360: ")); ROTATOR_SERIAL.println(currentStepsPerRevolution);

  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_POSITION_ADDR, storedPosition);
  EEPROM.put(EEPROM_SPEED_ADDR, currentMaxSpeed);
  EEPROM.put(EEPROM_ACCEL_ADDR, currentAccel);
  EEPROM.put(EEPROM_STEPS_ADDR, currentStepsPerRevolution);
  commitEEPROM();

  lastSavedPosition = currentPos;
  lastSavedSpeed = currentMaxSpeed;
  lastSavedAccel = currentAccel;
  lastSavedStepsPerRevolution = currentStepsPerRevolution;
  pendingPositionSave = false;

  ROTATOR_SERIAL.println(F("SAVED"));
}

void resetEEPROM() {
  ROTATOR_SERIAL.println(F("EEPROM: Resetting to defaults"));

  // Reset to defaults
  currentMaxSpeed = DEFAULT_SPEED;
  currentAccel = DEFAULT_ACCEL;
  currentStepsPerRevolution = DEFAULT_TOTAL_STEPS;
  positionAngleOffsetDeg = 0.0;
  rp2040.fifo.push(CMD_SET_POS   | 0);
  rp2040.fifo.push(CMD_SET_SPEED | (uint32_t)(int32_t)DEFAULT_SPEED);
  rp2040.fifo.push(CMD_SET_ACCEL | (uint32_t)(int32_t)DEFAULT_ACCEL);

  lastSavedSpeed = DEFAULT_SPEED;
  lastSavedAccel = DEFAULT_ACCEL;
  lastSavedStepsPerRevolution = DEFAULT_TOTAL_STEPS;
  lastSavedPosition = 0;
  pendingPositionSave = false;

  // Write defaults to EEPROM with valid magic number
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_POSITION_ADDR, 0L);
  EEPROM.put(EEPROM_SPEED_ADDR, DEFAULT_SPEED);
  EEPROM.put(EEPROM_ACCEL_ADDR, DEFAULT_ACCEL);
  EEPROM.put(EEPROM_STEPS_ADDR, DEFAULT_TOTAL_STEPS);
  commitEEPROM();

  ROTATOR_SERIAL.println(F("RESET"));
  ROTATOR_SERIAL.print(F("  Speed: ")); ROTATOR_SERIAL.println(DEFAULT_SPEED, 0);
  ROTATOR_SERIAL.print(F("  Accel: ")); ROTATOR_SERIAL.println(DEFAULT_ACCEL, 0);
  ROTATOR_SERIAL.print(F("  Steps/360: ")); ROTATOR_SERIAL.println(DEFAULT_TOTAL_STEPS);
}
