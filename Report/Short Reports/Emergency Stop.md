# HC‐SR04 Ultrasonic Emergency Stop: Design & Implementation
---
## Robot Configuration

The robot is built on an **Adafruit M0 Feather** microcontroller and is equipped with **four mecanum wheels**, allowing it to move in all directions, including sideways.  

An **ultrasonic distance sensor** (HC-SR04) is mounted at the front of the robot to detect obstacles and measure distances in real time. The microcontroller reads the sensor measurements and uses them to control the robot's motion.

---
## Task Description

The robot is programmed to perform an **emergency stop** when moving forward in directions **N (North), NW (North-West), or NE (North-East)**.  

Since the **ultrasonic sensor** is mounted at the front of the robot, it can detect obstacles only in these forward directions. When an obstacle is detected within the defined threshold, the robot immediately stops to prevent collisions.


---
## Sensor Functionality
The HC‐SR04 uses ultrasonic time‐of‐flight. [1]

A short trigger pulse makes the transmitter emit an $40 kHz$ burst.
The receiver listens for the echo reflected by obstacles.

The Arduino measures the echo pulse width $t$ and computes distance $d$ via the speed of sound $c \approx 343 \frac{\mathrm{m}}{\mathrm{s}}$.

$$
d = \frac{c \cdot t}{2} = \frac{343 \frac{\mathrm{m}}{\mathrm{s}} \cdot t}{2}
$$

With $t$ in microseconds, a common conversion used in the code is:

$$
d_\text{cm} \approx \frac{t}{58}
$$

---
## HC‐SR04 pin functions
| Pin  | Function [2] |
|:----:|:--------:|
| VCC  | +5 V supply for HC‐SR04 |
| GND  | Ground reference (common with Arduino) |
| TRIG | Digital pin 0 (output). 10 µs pulse to start measurement |
| ECHO | Digital pin 1 (input). High pulse width proportional to distance |

---
## Hysteresis
### In Physics [3]
In physics, **hysteresis** refers to a system’s dependence on its history. That is, the system’s response doesn’t just depend on the current input, but also on its past states. A classic example is a magnetic material:

- If you apply a magnetic field to iron, it becomes magnetized.
- When you reduce the field back to zero, the magnetization doesn’t immediately return to zero.
- The curve of magnetization and the applied field form a loop, showing that the material remembers its previous state.

So, hysteresis prevents sudden flipping of the system when inputs fluctuate around a threshold.

### Hysteresis In This System [4]

In this robot system:

- The robot stops if an obstacle is too close.
- The robot does not immediately resume motion if the distance increases slightly above the stop threshold, because sensor readings may fluctuate due to noise.

Using hysteresis, two distance thresholds are, for example, defined:

| Action | Distance Threshold |
|--------|------------------|
| Stop   | 25 cm            |
| Resume | 40 cm            |

The behavior is as follows:

1. The robot moves forward (N, NW, NE movements).
2. When an obstacle appears at 30 cm, the robot keeps moving (above the stop threshold).
3. When the obstacle reaches 25 cm, the robot stops.
4. If the obstacle moves back to 30 cm, the robot remains stopped (below the resume threshold).
5. Once the obstacle reaches 40 cm, the robot resumes motion.

The gap between the stop and resume thresholds prevents rapid oscillations when the measured distance fluctuates around a single value.

---
## Implementation

### Hysteresis
The emergency stop uses **hysteresis**, meaning there are two thresholds:
- `EMERGENCY_STOP_DISTANCE` - distance at which the robot stops.
- `EMERGENCY_RESUME_DISTANCE` - distance at which the robot resumes motion once the obstacle moves away.
### `loop()` Function Logic
In the `loop()` function, the distance is continuously measured using the ultrasonic sensor:

```cpp
long distance = getDistanceCM();
```
The emergency stop is only checked if the feature is activated and the robot is moving forward:
```cpp
if (emergencyActivated) {
    bool forwardMovement = movingNorth || movingNorthEast || movingNorthWest;
    if (forwardMovement) {
        if (distance < EMERGENCY_STOP_DISTANCE) emergencyStopped = true;
        else if (distance > EMERGENCY_RESUME_DISTANCE) emergencyStopped = false;

        if (emergencyStopped) {
            stopAllMotors();
            return;
        }
    }
}
```
`movingNorth`, `movingNorthEast`, and `movingNorthWest` are **flags** indicating the robot’s current forward movement.

If the measured distance is **below the stop threshold**, the robot stops immediately by calling:

```cpp
stopAllMotors();
```
If the distance later exceeds the resume threshold, `emergencyStopped` is set to false and the robot can resume motion.

### Distance Measurement

The distance is obtained from the ultrasonic sensor with the following function: [5][6]

```cpp
long getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  if (duration == 0) return 999; // timeout value

  long distance = duration / 58; // convert microseconds to cm
  return distance;
}
```
A 10 µs pulse triggers the sensor.

`pulseIn` measures the echo time.

Distance in centimeters is calculated by dividing the duration by 58.

### Feature Activation & Deactivation

The feature can be toggled via HTTP requests: [7]
```cpp
// Activate emergency stop
if (req.indexOf("on") >= 0) {
    emergencyActivated = true;
}

// Deactivate emergency stop
if (req.indexOf("off") >= 0) {
    emergencyActivated = false;
    emergencyStopped = false;
}

```
This allows the robot to switch the emergency stop on or off remotely.

---
## References

[1] Parallax Inc. Ultrasonic Distance Sensor Application Note., 2017.

[2] Elecfreaks. HC-SR04 Ultrasonic Ranging Module Datasheet., 2013.

[3] Mayergoyz, I. Mathematical Models of Hysteresis and Their Applications. Academic Press, 2003.

[4] Åström, K.J. & Murray, R.M. Feedback Systems: An Introduction for Scientists and Engineers. Princeton University Press, 2010.

[5] Banzi, M. & Shiloh, M. Getting Started with Arduino (3rd Edition). Maker Media, 2015.

[6] Arduino.cc. pulseIn() Function Reference. Arduino Documentation, 2020.

[7] Arduino.cc. WiFi101 / WiFiNINA HTTP Server Examples. Arduino Documentation, 2022.





