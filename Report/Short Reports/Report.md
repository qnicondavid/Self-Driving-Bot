# Robot Path Display Logic Report
---
## Robot Hardware
The robot utilizes an Arduino-based Adafruit M0 microcontroller, alongside 4 mecanum wheels to move in every direction.
The mecanum wheels allow the robot to move in any direction it is required to, back, forward, sideways, and to rotate.
## Task Description
The robot is programmed to record its movements using a pathing logic and display them on the GUI.
## Movement Logic Function
```cpp
private void stepRobot() {
	double Vy = (FL + FR + BL + BR) / 4.0;
	double Vx = (-FL + FR + BL - BR) / 4.0;
	double omega = (-FL + FR - BL + BR) / 4.0;
	Vx = Vx * -1.0;
	double scale = 0.1;
	heading += omega * scale;
	double globalX = Vx * Math.cos(heading) - Vy * Math.sin(heading);
	double globalY = Vx * Math.sin(heading) + Vy * Math.cos(heading);
	posX += globalX * scale;
	posY -= globalY * scale;
	path.add(new Point((int) posX, (int) posY));
}
```
## Function Explanation [1]
The movement and position of the robot is calculated within the `stepRobot()` method. It uses the four-wheel inputs.
  
- `FL` - Front left wheel. 
- `FR` - Front right wheel.
- `BL` - Back left wheel.
- `BR` - Back right wheel.

To calculate the following:
 
1. `Vy`   
- (forward/backwards) velocity variable that gets assigned the value of `(FL + FR + BL + BR) / 4.0`.   
- The wheels all spin in the same direction when wanting to move forward or backward.   
- We only need to add the speed and divide by 4 to get the average velocity.  
   		
 
3. `Vx`  
- (sideways/strafe) velocity variable that gets assigned the value of `((-FL + FR + BL - BR) / 4.0)*-1.0`.   
- Due to the unique design of the mecanum wheels, the robot can move from side to side.  
- 2 opposite corner wheels push backwards while the other 2 opposite corner wheels push forwards, allowing the robot to strafe.  
- We multiply Vx by -1.0 to match the coordinate convention.  
- We divide by 4 to get the average velocity again.
  
4. `omega`  
- rotational velocity variable that gets assigned the value of `(-FL + FR - BL + BR) / 4.0`.  
- The rotation comes from the difference between the left wheels and the right wheels.  
- When the left wheels and the right wheels spin in different directions, the robot rotates.  
- Divide by 4 to get the average velocity.  
   
 
			
Then, a scale variable is made to slow everything down so the robot doesn’t move across the screen too fast.

```cpp
heading += omega * scale;
```
  
Using these variables, the method converts the velocity variables `Vy` and `Vx` into our world's coordinate system using the current heading. 

```cpp
double globalX = Vx * Math.cos(heading) - Vy * Math.sin(heading);
double globalY = Vx * Math.sin(heading) + Vy * Math.cos(heading);
```
This formula is the 2D rotation matrix applied to a vector:

- The matrix rotates the local velocity vector by the robot’s heading angle.
- This ensures that “forward” (Vy) is always aligned with the robot’s current facing direction, not fixed to the screen axes

Then the movement is added to the robot's current position.

```cpp
posX += globalX * scale;
posY -= globalY * scale;
```
At the end, it stores both `posX` and `posY` in a constant List variable `path`. 
## References
[1] Gfrerrer, A. (2008). *Geometry and Kinematics of the Mecanum Wheel*. Graz University of Technology.
