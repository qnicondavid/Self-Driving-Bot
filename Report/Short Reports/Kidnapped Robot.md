# Kidnapped Robot
---
## Task Description
The robot is droped at a random position on the playing field away from the line, the task is for the robot to find his way back to the line all on its own.

## Kidnapped A Function
```cpp
void kidnappedA() {
  int fw[4];
  for(int i = 0; i < 4; i++)
    fw[i] = baseSpeed;
  robot.move(fw);
  while(!detectLine());
  robot.stopAll();
  delay(1000);
  robot.startPID();
  unsigned long start = millis();
  while(1) {
   pidControl();
   if(millis() - start > 10000)
    break; 
  }
  robot.stopPID();
}

bool detectLine() {
  int L =irLeft.readDigital();
  int R = irRight.readDigital();
  return L || R;
}
}
```

## Function Explanation
The function starts by indicating the robot to move forward.
  
The function `detectLine()` will then come to play, if the robot steps over a line,  
the function will return either the left sensors output or the rights in the form of a boolean.  
  
  
When this happens, the while loop `while(!detectLine());` will stop repeating, leting the following code run after a delay of 1000:
```cp
robot.startPID();
  unsigned long start = millis();
  while(1) {
   pidControl();
   if(millis() - start > 10000)
    break; 
  }
  robot.stopPID();
}
```
This will result in the PID starting up allowing the robot to start following the line, seting the robot back on the track.


## Kidnapped B Function
```cpp
void kidnappedB(int steps, int time1, int time2) {
  int f[4] = { baseSpeed, baseSpeed, baseSpeed, baseSpeed };
  while (1) {
    bool move = circleMovement(steps, time1, time2);
    if (move)
      break;
    robot.move(f);
    delay(20);
    robot.stopAll();
    delay(1000);
    steps = steps + 100;
    time2 = time2 + 20;
  }
  delay(1000);
  baseSpeed = 120;
  robot.startPID();
  unsigned long start = millis();
  while (1) {
    pidControl();
    if (millis() - start > 10000)
      break;
  }
  robot.stopPID();
}

bool circleMovement(int steps, int time1, int time2) {
  int cw[4] = { baseSpeed, -baseSpeed, baseSpeed, -baseSpeed };
  int e[4] = { baseSpeed, -baseSpeed, -baseSpeed, baseSpeed };
  for (int i = 0; i < steps; i++) {
    robot.stopAll();
    if (detectLine()) return 1;
    robot.move(cw);
    delay(time1);
    robot.stopAll();
    if (detectLine()) return 1;
    robot.move(e);
    delay(time2);
  }
  robot.stopAll();
  return 0;
}
```
To start, the `circleMovment()` function is defined, when called it reads 3 inputs `steps`, `time1` and `time2`.     
At the start of the function 2 int arrays are defined, `cw` and `e`.   
They both store diffrent values of the robots motors, each of them with the purpose of moving it in a diffrent direction, `e` has the values needed to move the robot to the right, and `cw` have the required values to rotate the robot to the right.    
  
A "for" loop is started, it repeats once for every `steps`.   
The loop begins by fully stoping the robot, after which it checks to see if the sensors detect a black line, if they do the function stops and it returns a 1.    
If the sensors dont detect a black line the loop continues by rotating the robot by using the `cw` array, after a delay of `time1` the robot stops rotating, after which it checks for a black line again.  
If a black line still isnt detected it then begins moving to the right side, after a delay of `time2` the robot stops moving to the right and the loop repeats.   
If the loop ends naturally and no line is detected the robot stops and the function returns 0.

An int array `f[4]` is defined, it stores the values of the motor speeds, all set to positive, when used will move the robot forward.

An infinite loop is started

```cp
while (1) {
    bool move = circleMovement(steps, time1, time2);
    if (move)
      break;
    robot.move(f);
    delay(20);
    robot.stopAll();
    delay(1000);
    steps = steps + 100;
    time2 = time2 + 20;
  }
```

It calls the `circleMovment()` function, moving the robot in a clockwise circle and returning 1 or 0 depending on weather it encountered a line or not, the value of this function is then stored in the variable `move`.

If the robot does not encounter a line, `move` will have the value of 0, the rest of the while loop plays out.  
After a short delay of 20, the robot stops completly.  
After another delay, the variables that dictate the radius of the circle made by the robot are increased and the loop starts again.  
  
If the robot does encountered a line, `move` will have the value set to 1, this breaks the loop.

The following code runs.

```cp
baseSpeed = 120;
  robot.startPID();
  unsigned long start = millis();
  while (1) {
    pidControl();
    if (millis() - start > 10000)
      break;
  }
  robot.stopPID();
}
```
The baseSpeed is set to 120, after which PID is started.
