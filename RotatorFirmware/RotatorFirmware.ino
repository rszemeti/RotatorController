// FastAccelStepper with Smart EEPROM Persistence
#include <FastAccelStepper.h>
#include <EEPROM.h>

// Motor Configuration
const int STEPS_PER_REV = 200;
const int GEAR_RATIO = 50;
const int MICROSTEPS = 10;
const long TOTAL_STEPS = (long)STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;

// Pin definitions
const int stepPin = 9;
const int dirPin = 8;
const int serialTxPin = 4;
const int serialRxPin = 5;
const unsigned long serialBaudRate = 19200;

#define ROTATOR_SERIAL Serial2

// EEPROM addresses
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_POSITION_ADDR = 4;
const int EEPROM_SPEED_ADDR = 8;
const int EEPROM_ACCEL_ADDR = 12;
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

// Panning state
bool isPanning = false;
int panDirection = 0;
unsigned long lastPanCommandTime = 0;
const unsigned long PAN_TIMEOUT = 1000;

// Position update and save intervals
unsigned long lastPositionUpdate = 0;
const unsigned long positionUpdateInterval = 200;
unsigned long lastPositionSave = 0;
const unsigned long positionSaveInterval = 5000;

// Track what's been saved to avoid unnecessary writes
long lastSavedPosition = 0;
float lastSavedSpeed = 0;
float lastSavedAccel = 0;
const long POSITION_SAVE_THRESHOLD = 50;

void setup() {
  ROTATOR_SERIAL.setTX(serialTxPin);
  ROTATOR_SERIAL.setRX(serialRxPin);
  ROTATOR_SERIAL.begin(serialBaudRate);
  
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
    lastSavedPosition = stepper->getCurrentPosition();
    
    ROTATOR_SERIAL.println(F("=== Rotator Ready (Smart EEPROM) ==="));
    ROTATOR_SERIAL.print(F("Steps/360: ")); ROTATOR_SERIAL.println(TOTAL_STEPS);
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
  // Watchdog check
  if (isPanning) {
    if (millis() - lastPanCommandTime > PAN_TIMEOUT) {
      stopPanning();
    }
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
      savePositionToEEPROM(currentPos);
      lastSavedPosition = currentPos;
    }
    lastPositionSave = millis();
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
    stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
    
    // Save position immediately on emergency stop
    long currentPos = stepper->getCurrentPosition();
    if (currentPos != lastSavedPosition) {
      savePositionToEEPROM(currentPos);
      lastSavedPosition = currentPos;
    }
  }
  // Check S but exclude SETPOS, STOP, and SAVE
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
  
  long targetSteps = (long)(targetAngle * TOTAL_STEPS / 360.0);
  
  ROTATOR_SERIAL.print(F("GO:")); 
  ROTATOR_SERIAL.println(targetAngle, 1);
  
  stepper->moveTo(targetSteps);
}

void setCurrentPosition(float angle) {
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  
  long newPos = (long)(angle * TOTAL_STEPS / 360.0);
  stepper->setCurrentPosition(newPos);
  
  // Always save immediately when position is manually set
  savePositionToEEPROM(newPos);
  lastSavedPosition = newPos;
  
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

void printPosition() {
  long currentSteps = stepper->getCurrentPosition();
  float currentAngle = (float)currentSteps * 360.0 / TOTAL_STEPS;
  
  while (currentAngle < 0) currentAngle += 360.0;
  while (currentAngle >= 360.0) currentAngle -= 360.0;
  
  ROTATOR_SERIAL.print(F("Position: "));
  ROTATOR_SERIAL.print(currentAngle, 2);
  ROTATOR_SERIAL.println(F("°"));
}

void printInfo() {
  ROTATOR_SERIAL.println(F("\n=== Settings ==="));
  ROTATOR_SERIAL.print(F("MaxSpeed: ")); ROTATOR_SERIAL.println(currentMaxSpeed, 0);
  ROTATOR_SERIAL.print(F("Accel: ")); ROTATOR_SERIAL.println(currentAccel, 0);
  ROTATOR_SERIAL.print(F("PanSpeed: ")); ROTATOR_SERIAL.println(PAN_CONSTANT_SPEED);
  ROTATOR_SERIAL.print(F("Steps/360: ")); ROTATOR_SERIAL.println(TOTAL_STEPS);
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
  float angle = (float)pos * 360.0 / TOTAL_STEPS;
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

// ========== EEPROM Functions ==========

void loadFromEEPROM() {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  
  ROTATOR_SERIAL.print(F("EEPROM Magic: 0x"));
  ROTATOR_SERIAL.println(magic, HEX);
  
  if (magic == EEPROM_MAGIC) {
    // Load position
    long savedPosition;
    EEPROM.get(EEPROM_POSITION_ADDR, savedPosition);
    stepper->setCurrentPosition(savedPosition);
    lastSavedPosition = savedPosition;
    
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
    }
    
    ROTATOR_SERIAL.println(F("EEPROM: Loaded"));
  } else {
    ROTATOR_SERIAL.println(F("EEPROM: No valid data, initializing defaults"));
    currentMaxSpeed = DEFAULT_SPEED;
    currentAccel = DEFAULT_ACCEL;
    stepper->setCurrentPosition(0);
    
    lastSavedSpeed = DEFAULT_SPEED;
    lastSavedAccel = DEFAULT_ACCEL;
    lastSavedPosition = 0;
    
    // Initialize EEPROM with valid data
    EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    EEPROM.put(EEPROM_POSITION_ADDR, 0L);
    EEPROM.put(EEPROM_SPEED_ADDR, DEFAULT_SPEED);
    EEPROM.put(EEPROM_ACCEL_ADDR, DEFAULT_ACCEL);
    ROTATOR_SERIAL.println(F("EEPROM: Initialized"));
  }
}

void savePositionToEEPROM(long position) {
  // EEPROM.put only writes if value changed (built-in optimization)
  EEPROM.put(EEPROM_POSITION_ADDR, position);
}

void saveSpeedToEEPROM(float speed) {
  EEPROM.put(EEPROM_SPEED_ADDR, speed);
}

void saveAccelToEEPROM(float accel) {
  EEPROM.put(EEPROM_ACCEL_ADDR, accel);
}

void forceSaveAll() {
  long currentPos = stepper->getCurrentPosition();
  
  ROTATOR_SERIAL.println(F("Force saving all to EEPROM:"));
  ROTATOR_SERIAL.print(F("  Position: ")); ROTATOR_SERIAL.println(currentPos);
  ROTATOR_SERIAL.print(F("  Speed: ")); ROTATOR_SERIAL.println(currentMaxSpeed, 0);
  ROTATOR_SERIAL.print(F("  Accel: ")); ROTATOR_SERIAL.println(currentAccel, 0);
  
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_POSITION_ADDR, currentPos);
  EEPROM.put(EEPROM_SPEED_ADDR, currentMaxSpeed);
  EEPROM.put(EEPROM_ACCEL_ADDR, currentAccel);
  
  lastSavedPosition = currentPos;
  lastSavedSpeed = currentMaxSpeed;
  lastSavedAccel = currentAccel;
  
  ROTATOR_SERIAL.println(F("SAVED"));
}

void resetEEPROM() {
  ROTATOR_SERIAL.println(F("EEPROM: Resetting to defaults"));
  
  // Reset to defaults
  currentMaxSpeed = DEFAULT_SPEED;
  currentAccel = DEFAULT_ACCEL;
  stepper->setCurrentPosition(0);
  stepper->setSpeedInHz(currentMaxSpeed);
  stepper->setAcceleration(currentAccel);
  
  lastSavedSpeed = DEFAULT_SPEED;
  lastSavedAccel = DEFAULT_ACCEL;
  lastSavedPosition = 0;
  
  // Write defaults to EEPROM with valid magic number
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_POSITION_ADDR, 0L);
  EEPROM.put(EEPROM_SPEED_ADDR, DEFAULT_SPEED);
  EEPROM.put(EEPROM_ACCEL_ADDR, DEFAULT_ACCEL);
  
  ROTATOR_SERIAL.println(F("RESET"));
  ROTATOR_SERIAL.print(F("  Speed: ")); ROTATOR_SERIAL.println(DEFAULT_SPEED, 0);
  ROTATOR_SERIAL.print(F("  Accel: ")); ROTATOR_SERIAL.println(DEFAULT_ACCEL, 0);
}