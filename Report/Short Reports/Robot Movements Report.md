# Robot Movements Report
---
## Robot Configuration
The robot is driven by four `DC` motors arranged in a mecanum-wheel configuration, allowing omnidirectional movement. 

Each motor is controlled by two pins: 
- `PWM` pin to set the speed (values between 0-255)
- `DIR` pin to set the rotation direction. 

Additionally, each motor has a boolean variable indicating whether a logical `HIGH` or `LOW` corresponds to forward rotation.

Pin mapping and polarity configuration for each motor:

| Motor              | PWM Pin | DIR Pin | Forward Polarity |
|:------------------:|:-------:|:-------:|:-----------------:|
| Front Left (FL)    |    6    |    5    |       false       |
| Front Right (FR)   |   11    |   12    |       true        |
| Back Left (BL)     |   A4    |   A5    |       false       |
| Back Right (BR)    |    9    |   10    |       true        |

A helper function is used to set each motor:
```cpp
void setMotor(int pwm, int dir, int speed, bool forward) {
  digitalWrite(dir, forward ? HIGH : LOW);
  analogWrite(pwm, speed);
}
```
The global variable `motorSpeed` determines the `PWM` duty cycle (default 80). 

Movements are implemented by applying appropriate direction patterns to each motor.

---
## Vertical Movements
Vertical movements correspond to forward (north) and backward (south) directions of the robot.

These movements involve all four mecanum wheels moving in the same direction.
### North (Forward)
To move the robot north, all four wheels rotate forward.

This movement sets the `movingNorth` flag to `true` for logic tracking, and then calls `setMotor` for each wheel.
```cpp
void moveNorth() {
  movingNorth = true;
  FL_speed = motorSpeed;
  FR_speed = motorSpeed;
  BL_speed = motorSpeed;
  BR_speed = motorSpeed;
  setMotor(FL_PWM, FL_DIR, motorSpeed,  FL_fw);
  setMotor(FR_PWM, FR_DIR, motorSpeed,  FR_fw);
  setMotor(BL_PWM, BL_DIR, motorSpeed,  BL_fw);
  setMotor(BR_PWM, BR_DIR, motorSpeed,  BR_fw);
}
```

### South (Backward)
To move the robot south, all four wheels rotate backward.
```cpp
void moveSouth() {
  FL_speed = -motorSpeed;
  FR_speed = -motorSpeed;
  BL_speed = -motorSpeed;
  BR_speed = -motorSpeed;
  setMotor(FL_PWM, FL_DIR, motorSpeed, !FL_fw);
  setMotor(FR_PWM, FR_DIR, motorSpeed, !FR_fw);
  setMotor(BL_PWM, BL_DIR, motorSpeed, !BL_fw);
  setMotor(BR_PWM, BR_DIR, motorSpeed, !BR_fw);
}
```
---
## Horizontal Movements

Horizontal movements correspond to sideways strafing: moving right (east) or left (west).

With mecanum wheels, this is achieved by setting opposite diagonal wheels to rotate in opposite directions.

### East (Strafe Right)
To move the robot to the right:
- Front-left and rear-right wheels rotate forward.
- Front-right and rear-left wheels rotate backward.
```cpp
void moveEast() {
  FL_speed = motorSpeed;
  FR_speed = -motorSpeed;
  BL_speed = -motorSpeed;
  BR_speed = motorSpeed;
  setMotor(FL_PWM, FL_DIR, motorSpeed,  FL_fw);
  setMotor(FR_PWM, FR_DIR, motorSpeed, !FR_fw);
  setMotor(BL_PWM, BL_DIR, motorSpeed, !BL_fw);
  setMotor(BR_PWM, BR_DIR, motorSpeed,  BR_fw);
}
```
### West (Strafe Left)
To move the robot to the left:
- Front-left and rear-right wheels rotate backward.
- Front-right and rear-left wheels rotate forward.
```cpp
void moveWest() {
  FL_speed = -motorSpeed;
  FR_speed = motorSpeed;
  BL_speed = motorSpeed;
  BR_speed = -motorSpeed;
  setMotor(FL_PWM, FL_DIR, motorSpeed, !FL_fw);
  setMotor(FR_PWM, FR_DIR, motorSpeed,  FR_fw);
  setMotor(BL_PWM, BL_DIR, motorSpeed,  BL_fw);
  setMotor(BR_PWM, BR_DIR, motorSpeed, !BR_fw);
}
```
---
## Diagonal Movements

Diagonal movements allow the robot to move **northeast, northwest, southeast, or southwest**. 

With mecanum wheels, only the two wheels along the intended diagonal direction are powered, while the other two are stopped. 

This produces a smooth diagonal motion.  
### North-East (Forward-Right)

- **Front-left and rear-right wheels** move forward.  
- **Front-right and rear-left wheels** are stopped.  

```cpp
void moveNorthEast() {
  movingNorthEast = true;
  FL_speed = motorSpeed;
  FR_speed = 0;
  BL_speed = 0;
  BR_speed = motorSpeed;
  setMotor(FL_PWM, FL_DIR, motorSpeed,  FL_fw);
  setMotor(FR_PWM, FR_DIR, 0,           true);
  setMotor(BL_PWM, BL_DIR, 0,           true);
  setMotor(BR_PWM, BR_DIR, motorSpeed,  BR_fw);
}
```
### North-West (Forward-Left)
- **Front-right and rear-left wheels** move forward.
- **Front-left and rear-right wheels** are stopped.
```cpp
void moveNorthWest() {
  movingNorthWest = true;
  FL_speed = 0;
  FR_speed = motorSpeed;
  BL_speed = motorSpeed;
  BR_speed = 0;
  setMotor(FL_PWM, FL_DIR, 0,           true);
  setMotor(FR_PWM, FR_DIR, motorSpeed,  FR_fw);
  setMotor(BL_PWM, BL_DIR, motorSpeed,  BL_fw);
  setMotor(BR_PWM, BR_DIR, 0,           true);
}
```
### South-East (Backward-Right) 
- **Front-right and rear-left wheels** move backward.
- **Front-left and rear-right wheels** are stopped.
```cpp
void moveSouthEast() {
  FL_speed = 0;
  FR_speed = -motorSpeed;
  BL_speed = -motorSpeed;
  BR_speed = 0;
  setMotor(FL_PWM, FL_DIR, 0,            true);
  setMotor(FR_PWM, FR_DIR, motorSpeed,  !FR_fw);
  setMotor(BL_PWM, BL_DIR, motorSpeed,  !BL_fw);
  setMotor(BR_PWM, BR_DIR, 0,            true);
}
```
### South-West (Backward-Left)
- **Front-left and rear-right** wheels move backward.
- **Front-right and rear-left** wheels are stopped.
```cpp
void moveSouthWest() {
  FL_speed = -motorSpeed;
  FR_speed = 0;
  BL_speed = 0;
  BR_speed = -motorSpeed;
  setMotor(FL_PWM, FL_DIR, motorSpeed,  !FL_fw);
  setMotor(FR_PWM, FR_DIR, 0,            true);
  setMotor(BL_PWM, BL_DIR, 0,            true);
  setMotor(BR_PWM, BR_DIR, motorSpeed,  !BR_fw);
}
```

---
## Rotations

Rotational movements allow the robot to **turn clockwise (CW)** or **counter-clockwise (CCW)** in place. 

With mecanum wheels, rotation is achieved by spinning the left and right wheels in **opposite directions**.

### Rotate Clockwise (CW)

- **Front-left and rear-left wheels** move forward.  
- **Front-right and rear-right wheels** move backward.  
- This causes the robot to rotate around its center to the right.

```cpp
void rotateClockwise() {
  FL_speed = motorSpeed;
  FR_speed = -motorSpeed;
  BL_speed = motorSpeed;
  BR_speed = -motorSpeed;
  setMotor(FL_PWM, FL_DIR, motorSpeed,  FL_fw);
  setMotor(FR_PWM, FR_DIR, motorSpeed, !FR_fw);
  setMotor(BL_PWM, BL_DIR, motorSpeed,  BL_fw);
  setMotor(BR_PWM, BR_DIR, motorSpeed, !BR_fw);
}
```
### Rotate Counter-Clockwise (CCW)
- **Front-left and rear-left wheels** move backward.
- **Front-right and rear-right wheels** move forward.
- This causes the robot to rotate around its center to the left.
```cpp
void rotateCounterClockwise() {
  FL_speed = -motorSpeed;
  FR_speed = motorSpeed;
  BL_speed = -motorSpeed;
  BR_speed = motorSpeed;
  setMotor(FL_PWM, FL_DIR, motorSpeed, !FL_fw);
  setMotor(FR_PWM, FR_DIR, motorSpeed,  FR_fw);
  setMotor(BL_PWM, BL_DIR, motorSpeed, !BL_fw);
  setMotor(BR_PWM, BR_DIR, motorSpeed,  BR_fw);
}
```
