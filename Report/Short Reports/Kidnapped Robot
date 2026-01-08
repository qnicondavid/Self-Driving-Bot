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
