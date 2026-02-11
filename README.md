# Aerial Rotator Controller

A stepper motor based antenna rotator controller with GUI, designed for amateur radio use. The system uses an Arduino to drive a geared stepper motor, with a Python GUI for control and station management.

## Hardware

- Arduino (Uno, Nano, or similar)
- Stepper motor with driver (STEP/DIR interface)
- 50:1 gear reduction (configurable in firmware)
- 200 steps/rev motor with 10x microstepping = 100,000 steps per revolution

### Wiring

| Arduino Pin | Function |
|-------------|----------|
| 9 | STEP |
| 8 | DIR |

## Firmware

The Arduino firmware uses the FastAccelStepper library for smooth acceleration control. Settings are persisted to EEPROM so position, speed, and acceleration survive power cycles.

### Building

1. Install the FastAccelStepper library via Arduino Library Manager
2. Open `rotator.ino` in Arduino IDE
3. Adjust `GEAR_RATIO` and `MICROSTEPS` if your setup differs
4. Upload to your Arduino

### Serial Protocol

The firmware accepts commands at 115200 baud:

| Command | Description |
|---------|-------------|
| `A<angle>` | Go to angle (0-360) |
| `H` | Go home (0 degrees) |
| `P` | Print current position |
| `SETPOS<angle>` | Set current position without moving |
| `PANLEFT` | Start panning left (requires keepalive) |
| `PANRIGHT` | Start panning right (requires keepalive) |
| `PANSTOP` | Stop panning |
| `STOP` | Emergency stop |
| `S<speed>` | Set max speed (100-50000 steps/s) |
| `AC<accel>` | Set acceleration (100-100000 steps/s^2) |
| `SAVE` | Force save all settings to EEPROM |
| `LOAD` | Reload settings from EEPROM |
| `RESET` | Reset to factory defaults |
| `INFO` | Print current settings |

Pan commands have a 1 second watchdog timeout. The GUI sends keepalive commands while the pan button is held, so if the connection drops the motor stops automatically.

## GUI Application

The Python GUI provides a visual compass display, station database with bearing/distance calculations, and an HTTP API for integration with logging software.

### Requirements

```
pip install pyserial flask
```

Tkinter is included with most Python installations.

### Running

```
python rotator_gui.py
```

The GUI will attempt to reconnect to the last used serial port on startup.

### Features

- Compass rose display with current heading and target indicator
- Direct angle entry or quick buttons for cardinal directions
- Continuous pan with mouse (hold button) or keyboard (< and > keys)
- Mouse scroll wheel over compass to nudge target heading
- Station database with Maidenhead locator support
- Automatic bearing and distance calculation from your QTH
- Settings persisted between sessions

### Station Database

Set your own Maidenhead locator in Settings > Set My Location. Add stations with their callsign and locator, and the GUI will calculate bearing and distance. Double-click a station or click "Go To Selected Station" to point the antenna.

### HTTP API

The GUI runs a local web server (default port 5000) for integration with logging or contest software.

**GET /status**

Returns current rotator state:

```json
{
  "status": "running",
  "my_locator": "IO91wm",
  "stations_count": 5,
  "rotator_connected": true,
  "current_angle": 145.5
}
```

**POST /station**

Add or update a station:

```json
{
  "callsign": "G3XYZ",
  "locator": "IO92ab"
}
```

The API is CORS-enabled for browser access.

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `,` or `<` | Pan left (hold) |
| `.` or `>` | Pan right (hold) |
| Scroll wheel (over compass) | Adjust target heading |

## Configuration

### Firmware

Edit these constants in the .ino file to match your hardware:

```cpp
const int STEPS_PER_REV = 200;    // Motor steps per revolution
const int GEAR_RATIO = 50;         // Gearbox ratio
const int MICROSTEPS = 10;         // Driver microstepping setting
```

### GUI

Settings are stored in `rotator_settings.json` in the working directory:

- Last serial port
- Your Maidenhead locator
- Station database

Motion settings (speed, acceleration) are stored in the Arduino's EEPROM.

## Troubleshooting

**GUI shows "No Response - Disconnecting"**

The Arduino isn't responding. Check the serial port, baud rate, and that the firmware is uploaded correctly.

**Position drifts over time**

The system is open-loop. If you have limit switches or an encoder, you could add homing functionality to the firmware.

**EEPROM wearing out**

Position is only saved every 5 seconds and only if it changed by more than 50 steps. Speed and acceleration are only saved when they actually change. This should give years of use before EEPROM wear becomes an issue.

## License

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License (CC BY-SA 4.0).
You are free to share and adapt this work, provided you give appropriate credit and distribute any derivative works under the same license.
https://creativecommons.org/licenses/by-sa/4.0/