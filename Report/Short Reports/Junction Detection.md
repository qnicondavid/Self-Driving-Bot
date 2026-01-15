# Junction Detection System
---

## 1. System Overview

Junction detection operates within two primary modes:

- **Random Maze Solving**
  - Robot follows lines using PID control
  - Upon detecting a junction, it randomly selects a new direction by spinning for a randomized duration

- **Manual Maze Solving**
  - Robot executes a predefined queue of directional commands (`LEFT`, `FORWARD`, `RIGHT`)
  - Each junction triggers dequeuing and execution of the next command

In both modes:
- PID control runs continuously for line following
- Junction conditions are periodically checked
- When a junction is confirmed:
  1. PID control is suspended
  2. Robot nudges forward to center itself
  3. Directional maneuver is executed
  4. PID control resumes

---

## 2. Implementation Analysis

### 2.1 Core Detection Algorithm

The junction detection algorithm is implemented in the `inJunction()` function:

```cpp
bool inJunction() {
  int L = irLeft.readDigital();
  int R = irRight.readDigital();

  if (L && R) {
    junctionTimer++;
  } else {
    junctionTimer = 0;
    junctionDetected = false;
  }

  if (junctionTimer > JUNCTION_N && !junctionDetected) {
    junctionDetected = true;
    return true;
  }

  return false;
}
```

#### Key Variables

- **junctionTimer**
  - Integer counter incremented when both sensors detect a line
  - Resets to zero when the condition is lost

- **junctionDetected**
  - Boolean flag preventing multiple triggers for the same junction
  - Resets once both sensors no longer detect a line

- **JUNCTION_N**
  - Threshold constant set to `10`
  - Defines the minimum number of consecutive detections required

This **temporal filtering** approach reduces false positives caused by noise, surface irregularities, or brief anomalies.

### 2.2 Integration with Navigation

Junction detection is integrated into two navigation functions:
- `mazeSolving()`
- `manualMazeSolving()`

Both follow the same control flow:

1. **Continuous Monitoring**
   - `inJunction()` is called every loop iteration during PID line following

2. **Detection Response**
   - PID control is halted using `robot.stopPID()`

3. **Centering Maneuver**
   - `nudge()` drives the robot forward at base speed for 300 ms

4. **Directional Decision**
   - Random mode: `randomSpin()`
   - Manual mode: `goDirection()` using queued commands

5. **Recovery Mechanism**
   - `recoverDeadEnd()` ensures at least one sensor reacquires the line

6. **PID Resumption**
   - Line following resumes via `robot.startPID()`

The system then continues until the next junction is detected.
