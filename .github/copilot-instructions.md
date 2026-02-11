# Aerial Rotator Controller - AI Coding Guide

## System Architecture

This is a **dual-component amateur radio antenna rotator system**:
- **Firmware** ([RotatorFirmware/RotatorFirmware.ino](RotatorFirmware/RotatorFirmware.ino)): Arduino stepper motor controller with EEPROM persistence
- **GUI** ([GUI/RotatorGui.py](GUI/RotatorGui.py)): Python Tkinter desktop app with Flask REST API

Communication happens via serial protocol at 115200 baud. The GUI is the client; firmware is the command processor.

## Critical Design Patterns

### 1. Serial Protocol (115200 baud)
Commands are **uppercase, line-terminated strings**. Key examples from firmware:
- `A<angle>` - Go to angle (e.g., `A145.5`)
- `S<speed>` / `AC<accel>` - Set motion parameters
- `PANLEFT` / `PANRIGHT` - Continuous rotation (requires keepalive)
- `SETPOS<angle>` - Calibrate current position without moving
- `INFO` - Print debug information

**Order matters**: Check `AC` before `A` in command parsing to avoid false matches (see [RotatorFirmware.ino#L123-L131](RotatorFirmware/RotatorFirmware.ino#L123-L131)).

### 2. Watchdog Safety System
Pan commands (`PANLEFT`/`PANRIGHT`) have a **1-second timeout** (line 34). GUI sends keepalive commands every ~100ms while pan button is held. If connection drops, motor stops automatically. This prevents runaway rotation.

### 3. Smart EEPROM Persistence
Position is saved **only if changed by 50+ steps and every 5 seconds** to minimize wear (lines 104-112). Speed/acceleration save only when modified. Uses magic number `0xAE42` for validation.

### 4. Geometry Calculations
Uses **Maidenhead grid locator system** for ham radio positions. Functions `maidenhead_to_latlon()` and `calculate_bearing()` implement great-circle calculations (lines 13-67 in GUI). Bearing calculations require your QTH location set first.

### 5. Threading Architecture
GUI has **3 concurrent threads**:
- Main: Tkinter UI and canvas drawing
- Serial reader: Parses `Position: X.XX°` responses  
- Pan refresher: Sends keepalive commands during manual pan
- Flask server: HTTP API on port 5000 (optional, lazy-started)

## Hardware Constants (Firmware)

Edit these in `.ino` to match your hardware:
```cpp
const int STEPS_PER_REV = 200;    // Motor native steps
const int GEAR_RATIO = 50;         // Gearbox reduction
const int MICROSTEPS = 10;         // Driver microstepping
```
Total steps per 360° = `200 × 50 × 10 = 100,000`

## Development Workflows

### Building Firmware
Requires **FastAccelStepper** library (Arduino Library Manager). Upload to Arduino Uno/Nano using Arduino IDE or PlatformIO. No other dependencies.

### Running GUI
```powershell
pip install pyserial flask  # flask is optional for API
python GUI/RotatorGui.py
```
Settings persist to `rotator_settings.json` (last port, QTH locator, station database).

### Testing Serial Protocol
Use Serial Monitor at 115200 baud:
```
A90        # Go to 90 degrees
P          # Print position
INFO       # Show all settings
SAVE       # Force save to EEPROM
```

## Integration Points

### HTTP API (GUI)
Flask server runs on port 5000 with CORS enabled:
- `GET /status` - Returns rotator state and current angle
- `POST /station` - Add/update station in database (JSON: `{callsign, locator}`)

Used for integration with logging software (N1MM+, etc.).

### Canvas Rendering
GUI compass is a **600×600px canvas** with polar coordinate system. Target heading drawn at outer radius, current heading as arrow from center. Mouse wheel over canvas adjusts target by ±5°.

## Project-Specific Conventions

- **Angles are 0-360° continuous** (no wraparound in firmware position tracking until display)
- **Position updates print every 200ms** when moving (firmware line 97)
- **Capital letters for all serial commands** (firmware converts incoming to uppercase)
- GUI station list format: `CALLSIGN(12) LOCATOR(8) BEARING(6.1f)° DIST(5.0f)km`
- EEPROM addresses: Magic=0, Position=4, Speed=8, Accel=12
- Default motion: 4000 steps/s speed, 2000 steps/s² accel, 3000 steps/s pan speed

## Common Pitfalls

1. **Don't use `&&` for Arduino command chaining** - Serial is not a shell
2. **EEPROM changes need validation** - Always check magic number on load
3. **GUI disconnects show "No Response"** - Check timeout logic, not just serial errors
4. **Bearing calculations fail silently** - Requires `my_locator` set in GUI settings
5. **Pan commands must be uppercase** - `panleft` will be ignored

## Key Files

- [RotatorFirmware.ino](RotatorFirmware/RotatorFirmware.ino) - Full firmware, self-contained (473 lines)
- [RotatorGui.py](GUI/RotatorGui.py) - Complete GUI app (1487 lines, single file)
- [README.md](README.md) - User documentation with full protocol reference
