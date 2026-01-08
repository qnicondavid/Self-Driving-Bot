# Kidnapped Robot
---
## Task Description
The robot is droped at a random position on the playing field away from the line,   
the task is for the robot to find his way back to the line all on its own.  
## Playing field
The playing field is comprised of a piece of terrain with no track that is completly surrounded by black tape
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
This will result in the PID starting up allowing the robot to start following the line, seting back to robot onto the track.


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
