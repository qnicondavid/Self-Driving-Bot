# Turn Function
---
## Function
```cp
void Turn() {
  robot.stopAll();

  // Forward movement
  int x[4] = { baseSpeed, baseSpeed, baseSpeed, baseSpeed };

  // Rotation movement
  int y[4] = { baseSpeed, -baseSpeed, baseSpeed, -baseSpeed };

  // Apply direction multipliers
  for (int i = 0; i < 4; i++) {
    x[i] = x[i] * turnXDirection;
    y[i] = y[i] * turnYDirection;
  }

  // Initial forward motion
  robot.move(x);
  delay(turnTimeForwardStart);

  // Stepwise forward + rotation motion
  for (int i = 0; i < turnSteps; i++) {
    robot.move(x);
    delay(turnTimeForwardCorner);
    robot.move(y);
    delay(turnTimeRotationCorner);
  }

  // Final forward motion
  robot.move(x);
  delay(turnTimeForwardStart);
  robot.stopAll();
}
```
## Detailed Explenation
The function starts off by stoping all robot movment, after which it assigns to int arrays, one for forward movment `x` and one for rotational movment `y`.  
  
A small `for` loop is started with the purpose of changing the direction of the turn if the user so desires.  
  
Then the robot starts the movment, first it goes forward for a predetermined ammount of time (`turnTimeForwardStart`).  
  
The `for` loop that handles the turn starts, the length of the loop (`turnSteps`) is set in the gui, giving the user the ability to decide weather the turn should be a full U turn or a half turn. 



