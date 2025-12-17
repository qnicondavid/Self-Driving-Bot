# BackEnd.ino Changes Documentation

This document outlines all changes made to the original `BackEnd.ino` file.

---

## Summary of Changes

| Category | Changes Made |
|----------|--------------|
| Bug Fixes | 4 |
| New Features | 3 |
| Code Quality | 4 |

---

## Bug Fixes

### 1. Moved `pinMode()` Out of Constructors

**Problem:** Calling `pinMode()` inside class constructors is unsafe because global objects are constructed before the Arduino hardware is fully initialized.

**Original Code:**
```cpp
IrSensor(int aPin, int dPin) {
  analogPin = aPin;
  digitalPin = dPin;
  pinMode(analogPin, INPUT);   // Unsafe here
  pinMode(digitalPin, INPUT);  // Unsafe here
}
```

**Fixed Code:**
```cpp
IrSensor(int aPin, int dPin) : analogPin(aPin), digitalPin(dPin) {}

void begin() {
  pinMode(analogPin, INPUT);   // Safe - called from setup()
  pinMode(digitalPin, INPUT);
}
```

**Affected Classes:**
- `Microcontroller`
- `IrSensor`
- `UrSensor`
- `Motor`

All classes now have a `begin()` method, and `Robot::begin()` calls all of them.

---

### 2. Added `Serial.begin()`

**Problem:** The code called `Serial.print()` in `enableWiFi()` but never initialized Serial communication.

**Fix:** Added to `setup()`:
```cpp
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);  // SERIAL_BAUD_RATE = 115200
  delay(100);
  // ...
}
```

---

### 3. Fixed PID First-Run Spike

**Problem:** On the first PID iteration, `lastMillis` could be 0, causing an incorrect `dt` value and potentially a large derivative spike.

**Fix:** Added a `pidFirstRun` flag to skip the first iteration:
```cpp
bool pidFirstRun = true;

void pidControl() {
  // ...
  if (pidFirstRun) {
    lastMillis = now;
    pidFirstRun = false;
    return;  // Skip first iteration
  }
  // ...
}

void resetPID() {
  // ...
  pidFirstRun = true;
}
```

---

### 4. Added Derivative Clamping

**Problem:** If `dt` is very small, the derivative term could spike to extremely large values.

**Fix:** Added clamping to the derivative calculation:
```cpp
double derivative = (ef - previousError) / dt;
derivative = constrain(derivative, -1000.0, 1000.0);  // Prevent spikes
```

---

## New Features

### 1. Emergency Stop with Hysteresis

Implemented obstacle detection using the ultrasonic sensor with hysteresis to prevent oscillation.

**Constants:**
```cpp
#define EMERGENCY_STOP_DISTANCE   25    // cm - stop when closer
#define EMERGENCY_RESUME_DISTANCE 40    // cm - resume when farther
```

**New Methods in `Robot` class:**
```cpp
void setEmergencyActivated(bool activated);
bool isEmergencyActivated();
bool isEmergencyStopped();
bool checkEmergencyStop();
```

**Behavior:**
1. Robot stops when obstacle is closer than 25 cm
2. Robot remains stopped until obstacle is farther than 40 cm
3. The gap prevents rapid on/off switching when obstacle is near threshold

**Integration with PID:**
```cpp
void pidControl() {
  if (!robot.isInPID()) return;

  if (robot.checkEmergencyStop()) {
    return;  // Robot stopped due to obstacle
  }
  // ... rest of PID control
}
```

---

### 2. Full Movement Functions

Added all movement functions as documented in `Robot_Movements.md`:

| Function | Description |
|----------|-------------|
| `moveNorth()` | Forward |
| `moveSouth()` | Backward |
| `moveEast()` | Strafe right |
| `moveWest()` | Strafe left |
| `moveNorthEast()` | Diagonal forward-right |
| `moveNorthWest()` | Diagonal forward-left |
| `moveSouthEast()` | Diagonal backward-right |
| `moveSouthWest()` | Diagonal backward-left |
| `rotateClockwise()` | Rotate CW in place |
| `rotateCounterClockwise()` | Rotate CCW in place |

---

### 3. Extended HTTP API

Added new endpoints to `RequestHandler`:

**PID Control:**
- `GET /pid/on` — Start line following
- `GET /pid/off` — Stop line following

**Emergency Stop:**
- `GET /emergency/on` — Enable emergency stop
- `GET /emergency/off` — Disable emergency stop

**Movement:**
- `GET /move/north`
- `GET /move/south`
- `GET /move/east`
- `GET /move/west`
- `GET /move/northeast`
- `GET /move/northwest`
- `GET /move/southeast`
- `GET /move/southwest`
- `GET /move/cw` — Rotate clockwise
- `GET /move/ccw` — Rotate counter-clockwise
- `GET /stop` — Stop all motors

**Speed Control:**
- `GET /speed/{value}` — Set motor speed (0-255)

---

## Code Quality Improvements

### 1. Replaced Magic Numbers with Constants

All hardcoded values moved to `#define` statements at the top of the file:

```cpp
// WiFi Settings
#define WIFI_SSID       "Group 19"
#define WIFI_PASSWORD   "ShutDown1"

// Pin Definitions - Microcontroller
#define PIN_WIFI_CS     8
#define PIN_WIFI_IRQ    7
#define PIN_WIFI_RST    4
#define PIN_WIFI_EN     2

// Pin Definitions - IR Sensors
#define PIN_IR_LEFT_ANALOG   A1
#define PIN_IR_LEFT_DIGITAL  A0
#define PIN_IR_RIGHT_ANALOG  A3
#define PIN_IR_RIGHT_DIGITAL A2

// Pin Definitions - Ultrasonic Sensor
#define PIN_UR_TRIG     0
#define PIN_UR_ECHO     1

// Pin Definitions - Motors
#define PIN_FL_PWM      6
#define PIN_FL_DIR      5
// ... etc

// PID Settings
#define DEFAULT_BASE_SPEED    80
#define DEFAULT_MIN_SPEED     -120
#define DEFAULT_MAX_SPEED     120
#define DEFAULT_ALPHA         0.3
#define DEFAULT_KP            100.0
#define DEFAULT_KI            0.05
#define DEFAULT_KD            0.0
#define DEFAULT_MAX_INTEGRAL  200.0

// Timing Settings
#define STREAM_INTERVAL_MS    500
#define SERIAL_BAUD_RATE      115200
#define PULSE_TIMEOUT_US      30000
```

---

### 2. Renamed `printHandler()` to `handleClients()`

**Reason:** The function handles both streaming data and control requests, not just printing. The new name better reflects its purpose.

**Change:**
```cpp
// Old
void printHandler() { ... }

// New
void handleClients() { ... }
```

---

### 3. Improved Client Handling Logic

**Original:** Required two loop iterations to fill both client slots.

**Improved:** Accepts new client and assigns to first available slot, or replaces control client if both full:
```cpp
void handleClients() {
  WiFiClient newClient = mcu.getServer().available();
  
  if (newClient) {
    if (!streamClient || !streamClient.connected()) {
      streamClient = newClient;
    } else if (!controlClient || !controlClient.connected()) {
      controlClient = newClient;
    } else {
      controlClient = newClient;  // Replace control client
    }
  }
  // ...
}
```

---

### 4. Enhanced Diagnostics Output

`printInformation()` now includes PID and emergency stop status:

```cpp
void printInformation(Print &out) {
  // ... sensor readings ...
  // ... motor speeds ...
  
  out.println("=== Status ===");
  out.print("PID Mode: ");
  out.println(inPID ? "ON" : "OFF");
  out.print("Emergency Stop Active: ");
  out.println(emergencyActivated ? "YES" : "NO");
  out.print("Emergency Stopped: ");
  out.println(emergencyStopped ? "YES" : "NO");
}
```

---

## File Structure

The corrected file is organized into clear sections:

1. **Configuration Constants** — All `#define` statements
2. **Classes** — `Microcontroller`, `IrSensor`, `UrSensor`, `Motor`, `Robot`, `RequestHandler`
3. **Global Objects** — Instance declarations
4. **PID Variables** — PID state variables
5. **Functions** — `resetPID()`, `pidControl()`, `handleClients()`
6. **Arduino Entry Points** — `setup()`, `loop()`

---

## Notes

Emergency stop is **enabled by default** — use `/emergency/off` to disable if needed
