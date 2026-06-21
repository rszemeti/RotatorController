// FastAccelStepper with Smart EEPROM Persistence
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <EEPROM.h>

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

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Current settings
float currentMaxSpeed = DEFAULT_SPEED;
float currentAccel = DEFAULT_ACCEL;
long currentStepsPerRevolution = DEFAULT_TOTAL_STEPS;
float positionAngleOffsetDeg = 0.0;

// Panning state
bool isPanning = false;
int panDirection = 0;
unsigned long lastPanCommandTime = 0;
const unsigned long PAN_TIMEOUT = 1000;
bool hasActiveTarget = false;
long activeTargetSteps = 0;
const long ACTIVE_TARGET_DEADBAND_STEPS = 20;

// Position update and save intervals
unsigned long lastPositionUpdate = 0;
const unsigned long positionUpdateInterval = 200;
unsigned long lastPositionSave = 0;
const unsigned long positionSaveInterval = 5000;
const unsigned long positionIdleForceSaveInterval = 30000;
unsigned long lastMotionForPositionSave = 0;
bool pendingPositionSave = false;
long pendingPositionSteps = 0;

// External drive-disable pin control (active HIGH disables, LOW enables)
const unsigned long driveEnableSettleMs = 500;
const unsigned long driveIdleDisableMs = 10000;
bool driveEnabled = false;
unsigned long lastMotionActivityMs = 0;

// Heartbeat LED (1 Hz)
const unsigned long heartbeatIntervalMs = 500;
unsigned long lastHeartbeatToggle = 0;
bool heartbeatState = false;
const int heartbeatFallbackPin = 25;
const unsigned long usbStatusIntervalMs = 1000;
unsigned long lastUsbStatus = 0;

// Track what's been saved to avoid unnecessary writes
long lastSavedPosition = 0;
float lastSavedSpeed = 0;
float lastSavedAccel = 0;
long lastSavedStepsPerRevolution = DEFAULT_TOTAL_STEPS;
const long POSITION_SAVE_THRESHOLD = 50;

// Forward declarations (required for .cpp builds)
void processCommand(String command);
void startPanning(int direction);
void stopPanning();
void goToAngle(float targetAngle);
void setCurrentPosition(float angle);
void setSpeed(float speed);
void setAcceleration(float accel);
void setStepsPerRevolution(long steps);
long calculateNearestTargetSteps(float targetAngle, long referencePos);
void updateActiveTargetControl();
float normalizeAngleDeg(float angle);
void printPosition();
void printInfo();
float getCurrentAngle();
void loadFromEEPROM();
void setDriveEnabled(bool enabled);
void prepareDriveForMotion();
void updateDriveIdleState();
void savePositionToEEPROM(long position);
void saveSpeedToEEPROM(float speed);
void saveAccelToEEPROM(float accel);
void saveStepsToEEPROM(long steps);
void commitEEPROM();
void forceSaveAll();
void resetEEPROM();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(heartbeatFallbackPin, OUTPUT);
  pinMode(driveDisablePin, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(heartbeatFallbackPin, LOW);
  setDriveEnabled(false);

  Serial.begin(usbBaudRate);
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart < 2000)) {
    delay(10);
  }
  if (Serial) {
    Serial.println(F("BOOT OK"));
  }

  ROTATOR_SERIAL.setTX(serialTxPin);
  ROTATOR_SERIAL.setRX(serialRxPin);
  ROTATOR_SERIAL.begin(serialBaudRate);

  EEPROM.begin(EEPROM_SIZE_BYTES);

  engine.init();

  stepper = engine.stepperConnectToPin(stepPin);
  if (stepper) {
    stepper->setDirectionPin(dirPin);
    stepper->setAutoEnable(true);

    // Load saved settings from EEPROM
    loadFromEEPROM();

    // Apply loaded settings
    stepper->setSpeedInHz(currentMaxSpeed);
    stepper->setAcceleration(currentAccel);

    // Initialize "last saved" trackers
    lastSavedSpeed = currentMaxSpeed;
    lastSavedAccel = currentAccel;
    lastSavedStepsPerRevolution = currentStepsPerRevolution;
    lastSavedPosition = stepper->getCurrentPosition();

    ROTATOR_SERIAL.println(F("=== Rotator Ready (Smart EEPROM) ==="));
    ROTATOR_SERIAL.print(F("Loaded Steps/360: ")); ROTATOR_SERIAL.println(currentStepsPerRevolution);
    ROTATOR_SERIAL.print(F("Loaded Position: ")); ROTATOR_SERIAL.print(getCurrentAngle(), 2); ROTATOR_SERIAL.println(F("°"));
    ROTATOR_SERIAL.print(F("Loaded Speed: ")); ROTATOR_SERIAL.println(currentMaxSpeed, 0);
    ROTATOR_SERIAL.print(F("Loaded Accel: ")); ROTATOR_SERIAL.println(currentAccel, 0);
    ROTATOR_SERIAL.println(F("GUI Ready"));

    printPosition();
  } else {
    ROTATOR_SERIAL.println(F("ERROR: Stepper init failed!"));
  }
}

void loop() {
  if (millis() - lastHeartbeatToggle >= heartbeatIntervalMs) {
    heartbeatState = !heartbeatState;
    digitalWrite(LED_BUILTIN, heartbeatState ? HIGH : LOW);
    digitalWrite(heartbeatFallbackPin, heartbeatState ? HIGH : LOW);
    lastHeartbeatToggle = millis();
  }

  if (Serial && (millis() - lastUsbStatus >= usbStatusIntervalMs)) {
    Serial.println(F("ALIVE"));
    lastUsbStatus = millis();
  }

  // Watchdog check
  if (isPanning) {
    if (millis() - lastPanCommandTime > PAN_TIMEOUT) {
      stopPanning();
    }
  }

  updateDriveIdleState();

  updateActiveTargetControl();

  if (isPanning || stepper->isRunning()) {
    lastMotionForPositionSave = millis();
  }

  // Position updates
  if (millis() - lastPositionUpdate >= positionUpdateInterval) {
    if (isPanning || stepper->isRunning()) {
      printPosition();
    }
    lastPositionUpdate = millis();
  }

  // Periodic position save (only if changed significantly)
  if (millis() - lastPositionSave >= positionSaveInterval) {
    long currentPos = stepper->getCurrentPosition();
    if (abs(currentPos - lastSavedPosition) >= POSITION_SAVE_THRESHOLD) {
      if (isPanning || stepper->isRunning()) {
        // Defer flash writes while moving to avoid periodic motion stutter.
        pendingPositionSave = true;
        pendingPositionSteps = currentPos;
      } else {
        savePositionToEEPROM(currentPos);
        lastSavedPosition = currentPos;
      }
    }
    lastPositionSave = millis();
  }

  // Flush deferred position save once motion is idle.
  if (pendingPositionSave && !isPanning && !stepper->isRunning()) {
    savePositionToEEPROM(pendingPositionSteps);
    lastSavedPosition = pendingPositionSteps;
    pendingPositionSave = false;
  }

  // Fallback save: if position changed and has been idle for a while, persist once.
  if (!isPanning && !stepper->isRunning() &&
      (millis() - lastMotionForPositionSave >= positionIdleForceSaveInterval)) {
    long currentPos = stepper->getCurrentPosition();
    if (currentPos != lastSavedPosition) {
      savePositionToEEPROM(currentPos);
      lastSavedPosition = currentPos;
    }
  }

  // Process commands
  if (ROTATOR_SERIAL.available() > 0) {
    String command = ROTATOR_SERIAL.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    processCommand(command);
  }
}

void processCommand(String command) {
  // Check AC BEFORE A (otherwise AC2000 gets parsed as A with "C2000")
  if (command.startsWith("AC")) {
    float accel = command.substring(2).toFloat();
    setAcceleration(accel);
  }
  else if (command.startsWith("A")) {
    stopPanning();
    float angle = command.substring(1).toFloat();
    goToAngle(angle);
  }
  else if (command == "H") {
    stopPanning();
    goToAngle(0);
  }
  else if (command == "P") {
    printPosition();
  }
  else if (command.startsWith("SETPOS")) {
    stopPanning();
    float angle = command.substring(6).toFloat();
    setCurrentPosition(angle);
  }
  else if (command == "PANLEFT") {
    if (isPanning && panDirection == -1) {
      lastPanCommandTime = millis();
    } else {
      startPanning(-1);
    }
  }
  else if (command == "PANRIGHT") {
    if (isPanning && panDirection == 1) {
      lastPanCommandTime = millis();
    } else {
      startPanning(1);
    }
  }
  else if (command == "PANSTOP") {
    stopPanning();
  }
  else if (command == "STOP") {
    stopPanning();
    hasActiveTarget = false;
    stepper->forceStopAndNewPosition(stepper->getCurrentPosition());

    // Save position immediately on emergency stop
    long currentPos = stepper->getCurrentPosition();
    if (currentPos != lastSavedPosition) {
      savePositionToEEPROM(currentPos);
      lastSavedPosition = currentPos;
    }
  }
  // Check S but exclude SETPOS, STOP, and SAVE
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


void startPanning(int direction) {
  hasActiveTarget = false;
  prepareDriveForMotion();
  isPanning = true;
  panDirection = direction;
  lastPanCommandTime = millis();

  stepper->setSpeedInHz(PAN_CONSTANT_SPEED);

  if (direction > 0) {
    stepper->runForward();
  } else {
    stepper->runBackward();
  }

  ROTATOR_SERIAL.print(F("PAN:"));
  ROTATOR_SERIAL.println(direction > 0 ? 'R' : 'L');
}

void stopPanning() {
  if (isPanning) {
    isPanning = false;
    panDirection = 0;
    stepper->stopMove();

    stepper->setSpeedInHz(currentMaxSpeed);

    // Save position after panning (only if changed)
    long currentPos = stepper->getCurrentPosition();
    if (abs(currentPos - lastSavedPosition) >= POSITION_SAVE_THRESHOLD) {
      savePositionToEEPROM(currentPos);
      lastSavedPosition = currentPos;
    }
  }
}

void goToAngle(float targetAngle) {
  while (targetAngle < 0) targetAngle += 360.0;
  while (targetAngle >= 360.0) targetAngle -= 360.0;

  long currentPos = stepper->getCurrentPosition();
  activeTargetSteps = calculateNearestTargetSteps(targetAngle, currentPos);
  hasActiveTarget = true;
  float currentAngle = getCurrentAngle();
  long deltaSteps = activeTargetSteps - currentPos;

  ROTATOR_SERIAL.print(F("GO:"));
  ROTATOR_SERIAL.println(targetAngle, 1);
  ROTATOR_SERIAL.print(F("GO DBG cur="));
  ROTATOR_SERIAL.print(currentPos);
  ROTATOR_SERIAL.print(F(" curAng="));
  ROTATOR_SERIAL.print(currentAngle, 2);
  ROTATOR_SERIAL.print(F(" dAng="));
  ROTATOR_SERIAL.print(targetAngle - currentAngle, 2);
  ROTATOR_SERIAL.print(F(" dStp="));
  ROTATOR_SERIAL.print(deltaSteps);
  ROTATOR_SERIAL.print(F(" tgt="));
  ROTATOR_SERIAL.print(activeTargetSteps);
  ROTATOR_SERIAL.print(F(" spr="));
  ROTATOR_SERIAL.println(currentStepsPerRevolution);

  if (deltaSteps != 0) {
    prepareDriveForMotion();
    stepper->moveTo(activeTargetSteps);
  }
}

long calculateNearestTargetSteps(float targetAngle, long referencePos) {
  float relativeTargetAngle = normalizeAngleDeg(targetAngle - positionAngleOffsetDeg);
  long targetStepsBase = (long)round((double)relativeTargetAngle * (double)currentStepsPerRevolution / 360.0);
  long nearestTurn = (long)round((double)(referencePos - targetStepsBase) / (double)currentStepsPerRevolution);
  return targetStepsBase + nearestTurn * currentStepsPerRevolution;
}

float normalizeAngleDeg(float angle) {
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

void updateActiveTargetControl() {
  if (!hasActiveTarget || isPanning) {
    return;
  }

  long currentPos = stepper->getCurrentPosition();
  if (abs(activeTargetSteps - currentPos) <= ACTIVE_TARGET_DEADBAND_STEPS) {
    hasActiveTarget = false;
    return;
  }

  if (!stepper->isRunning()) {
    prepareDriveForMotion();
    stepper->moveTo(activeTargetSteps);
  }
}

void setDriveEnabled(bool enabled) {
  pinMode(driveDisablePin, OUTPUT);
  digitalWrite(driveDisablePin, enabled ? LOW : HIGH);
  driveEnabled = enabled;
}

void prepareDriveForMotion() {
  if (!driveEnabled) {
    setDriveEnabled(true);
    delay(driveEnableSettleMs);
  }
  lastMotionActivityMs = millis();
}

void updateDriveIdleState() {
  if (isPanning || stepper->isRunning()) {
    lastMotionActivityMs = millis();
    return;
  }

  if (driveEnabled && (millis() - lastMotionActivityMs >= driveIdleDisableMs)) {
    setDriveEnabled(false);
  }
}

void setCurrentPosition(float angle) {
  angle = normalizeAngleDeg(angle);

  long currentSteps = stepper->getCurrentPosition();
  float relativeAngle = (float)currentSteps * 360.0 / currentStepsPerRevolution;
  positionAngleOffsetDeg = normalizeAngleDeg(angle - relativeAngle);
  hasActiveTarget = false;

  // Always save immediately when position is manually set
  savePositionToEEPROM(currentSteps);
  lastSavedPosition = currentSteps;

  ROTATOR_SERIAL.print(F("SET:"));
  ROTATOR_SERIAL.println(angle, 1);
}

void setSpeed(float speed) {
  if (speed < 100) {
    ROTATOR_SERIAL.println(F("Speed too low, setting to 100"));
    speed = 100;
  }
  if (speed > 50000) {
    ROTATOR_SERIAL.println(F("Speed too high, limiting to 50000"));
    speed = 50000;
  }

  currentMaxSpeed = speed;
  stepper->setSpeedInHz(speed);

  // Only save if value actually changed
  if (abs(speed - lastSavedSpeed) > 1.0) {
    saveSpeedToEEPROM(speed);
    lastSavedSpeed = speed;
  }

  ROTATOR_SERIAL.print(F("SPD:"));
  ROTATOR_SERIAL.println(speed, 0);
}

void setAcceleration(float accel) {
  if (accel < 100) {
    ROTATOR_SERIAL.println(F("Accel too low, setting to 100"));
    accel = 100;
  }
  if (accel > 100000) {
    ROTATOR_SERIAL.println(F("Accel too high, limiting to 100000"));
    accel = 100000;
  }

  currentAccel = accel;
  stepper->setAcceleration(accel);

  // Only save if value actually changed
  if (abs(accel - lastSavedAccel) > 1.0) {
    saveAccelToEEPROM(accel);
    lastSavedAccel = accel;
  }

  ROTATOR_SERIAL.print(F("ACC:"));
  ROTATOR_SERIAL.println(accel, 0);
}

void setStepsPerRevolution(long steps) {
  if (steps < 1000) {
    ROTATOR_SERIAL.println(F("Steps/360 too low, setting to 1000"));
    steps = 1000;
  }
  if (steps > 10000000) {
    ROTATOR_SERIAL.println(F("Steps/360 too high, limiting to 10000000"));
    steps = 10000000;
  }

  if (isPanning || stepper->isRunning()) {
    ROTATOR_SERIAL.println(F("Cannot change Steps/360 while moving"));
    return;
  }

  long oldStepsPerRevolution = currentStepsPerRevolution;
  if (steps == oldStepsPerRevolution) {
    ROTATOR_SERIAL.print(F("STEPS:"));
    ROTATOR_SERIAL.println(steps);
    return;
  }

  long currentPos = stepper->getCurrentPosition();
  float currentAngle = getCurrentAngle();
  float relativeAngle = normalizeAngleDeg(currentAngle - positionAngleOffsetDeg);

  long remappedBase = (long)round((double)relativeAngle * (double)steps / 360.0);
  long nearestTurn = (long)round((double)(currentPos - remappedBase) / (double)steps);
  long remappedPos = remappedBase + nearestTurn * steps;
  stepper->setCurrentPosition(remappedPos);

  currentStepsPerRevolution = steps;
  savePositionToEEPROM(remappedPos);
  lastSavedPosition = remappedPos;

  if (steps != lastSavedStepsPerRevolution) {
    saveStepsToEEPROM(steps);
    lastSavedStepsPerRevolution = steps;
  }

  ROTATOR_SERIAL.print(F("STEPS:"));
  ROTATOR_SERIAL.println(steps);
}

void printPosition() {
  float currentAngle = getCurrentAngle();

  ROTATOR_SERIAL.print(F("Position: "));
  ROTATOR_SERIAL.print(currentAngle, 2);
  ROTATOR_SERIAL.println(F("°"));
}

void printInfo() {
  ROTATOR_SERIAL.println(F("\n=== Settings ==="));
  ROTATOR_SERIAL.print(F("MaxSpeed: ")); ROTATOR_SERIAL.println(currentMaxSpeed, 0);
  ROTATOR_SERIAL.print(F("Accel: ")); ROTATOR_SERIAL.println(currentAccel, 0);
  ROTATOR_SERIAL.print(F("PanSpeed: ")); ROTATOR_SERIAL.println(PAN_CONSTANT_SPEED);
  ROTATOR_SERIAL.print(F("Steps/360: ")); ROTATOR_SERIAL.println(currentStepsPerRevolution);
  ROTATOR_SERIAL.print(F("Panning: ")); ROTATOR_SERIAL.println(isPanning ? F("YES") : F("NO"));

  // Show EEPROM status
  ROTATOR_SERIAL.print(F("Position delta: "));
  ROTATOR_SERIAL.print(abs(stepper->getCurrentPosition() - lastSavedPosition));
  ROTATOR_SERIAL.print(F(" (saves at ")); ROTATOR_SERIAL.print(POSITION_SAVE_THRESHOLD); ROTATOR_SERIAL.println(F(")"));

  ROTATOR_SERIAL.print(F("Speed saved: "));
  ROTATOR_SERIAL.println(abs(currentMaxSpeed - lastSavedSpeed) < 1.0 ? F("YES") : F("NO"));

  ROTATOR_SERIAL.print(F("Accel saved: "));
  ROTATOR_SERIAL.println(abs(currentAccel - lastSavedAccel) < 1.0 ? F("YES") : F("NO"));

  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  ROTATOR_SERIAL.print(F("EEPROM Magic: 0x"));
  ROTATOR_SERIAL.println(magic, HEX);

  printPosition();
}

float getCurrentAngle() {
  long pos = stepper->getCurrentPosition();
  float relativeAngle = (float)pos * 360.0 / currentStepsPerRevolution;
  return normalizeAngleDeg(relativeAngle + positionAngleOffsetDeg);
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
  long currentPos = stepper->getCurrentPosition();
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
  stepper->setCurrentPosition(0);
  stepper->setSpeedInHz(currentMaxSpeed);
  stepper->setAcceleration(currentAccel);

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
