import javafx.animation.AnimationTimer;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

import java.util.ArrayList;


/**
 * MecanumVisualizer is a JavaFX Canvas that visualizes a robot with Mecanum wheels.
 * <p>
 * It tracks the robot's position, heading, motor states, and draws its path
 * in real time. The robot image rotates according to the heading, and the path
 * visualizes the robot's trajectory.
 */
public class MecanumVisualizer extends Canvas {
	
	// Motor PWM values
    private int FL, FR, BL, BR;
	
	// Robot position and orientation
    private double posX, posY;
    private double heading = 0.0;
	
	// Robot drawing constants
    private double ROBOT_RADIUS = 70.0;
    private double MOTOR_SCALE = 0.2; // Scale factor for motor input effect
    private double OMEGA_SCALE = 0.02; // Scale factor for rotation speed
    private double MOVE_SCALE = 0.05; // Scale factor for linear movement
	
	// Stores the path of the robot for visualization
    private final ArrayList<double[]> path = new ArrayList<>();

	// Image of the robot
    private final Image robotImage;


    /**
     * Constructs a MecanumVisualizer with specified canvas width and height.
     * Loads the robot image and starts the animation loop.
     *
     * @param width  canvas width
     * @param height canvas height
     */
    public MecanumVisualizer(double width, double height) {
        super(width, height);
		
		 // Initialize robot in the center
        positionRobot();
		
		// Load the robot image scaled to robot size
        robotImage = new Image(getClass().getResource("/robot.png").toExternalForm(),
                               ROBOT_RADIUS * 2, ROBOT_RADIUS * 2, true, true);
		
		// Animation timer for stepping and drawing robot continuously
        new AnimationTimer() {
            @Override
            public void handle(long now) {
                stepRobot(); // Update robot's position based on motors
                draw(); // Draw robot and path
            }
        }.start();
    }
	
	/** Sets the motor scale factor. */
	public void setMotorScale(double motorScale) {
		this.MOTOR_SCALE = motorScale;
	}
	
	/** Sets the omega (rotation) scale factor. */
	public void setOmegaScale(double omegaScale) {
		this.OMEGA_SCALE = omegaScale;
	}
	
	/** Sets the linear movement scale factor. */
	public void setMoveScale(double moveScale) {
		this.MOVE_SCALE = moveScale;
	}


    /**
     * Updates the current motor PWM values.
     *
     * @param fl front-left motor PWM
     * @param fr front-right motor PWM
     * @param bl back-left motor PWM
     * @param br back-right motor PWM
     */
    public void updateMotors(int fl, int fr, int bl, int br) {
        this.FL = fl;
        this.FR = fr;
        this.BL = bl;
        this.BR = br;
    }
	
	/** Initializes robot position in the center and resets path. */
    private void positionRobot() {
        posX = 390;
        posY = 390;
        path.clear();
        path.add(new double[]{posX, posY});
    }

	/**
     * Steps the robot simulation.
     *
     * Calculates linear (vx, vy) and rotational (omega) velocities
     * from Mecanum wheel motor values using a simplified model.
     * Updates heading and position, constraining within canvas bounds,
     * and adds the new position to the path.
     */
    public void stepRobot() {
        double vy = (FL + FR + BL + BR) / 4.0 * MOTOR_SCALE; // forward/backward
        double vx = (FL - FR - BL + BR) / 4.0 * MOTOR_SCALE; // sideways
        double omega = (-FL + FR - BL + BR) / 4.0 * MOTOR_SCALE; // rotation

        heading += omega * OMEGA_SCALE;
		
		// Rotate velocities to global coordinate frame
        double dx = vx * Math.cos(heading) - vy * Math.sin(heading);
        double dy = vx * Math.sin(heading) + vy * Math.cos(heading);

		// Update position
        posX += dx * MOVE_SCALE;
        posY -= dy * MOVE_SCALE; // minus because y-axis points downward in canvas

		// Constrain robot inside canvas
        posX = Math.max(ROBOT_RADIUS, Math.min(getWidth() - ROBOT_RADIUS, posX));
        posY = Math.max(ROBOT_RADIUS, Math.min(getHeight() - ROBOT_RADIUS, posY));

		// Add position to path
        path.add(new double[]{posX, posY});
        if (path.size() > 3000) path.remove(0); // limit path length
    }

	
    /**
     * Draws the robot and its path on the canvas.
     *
     * Clears the canvas, draws the path as a line, and draws the rotated robot image.
     */
    public void draw() {
        GraphicsContext g = getGraphicsContext2D();

		// Clear canvas with background color
        g.setFill(Color.web("#151515"));
        g.fillRect(0, 0, getWidth(), getHeight());

		// Draw path
        g.setStroke(Color.web("#0f9ed5"));
        g.setLineWidth(2);
        for (int i = 0; i < path.size() - 1; i++) {
            double[] p1 = path.get(i);
            double[] p2 = path.get(i + 1);
            g.strokeLine(p1[0], p1[1], p2[0], p2[1]);
        }
		// Save current transform
        Affine old = g.getTransform();
		
		// Translate and rotate for robot drawing
        g.translate(posX, posY);
        g.rotate(-Math.toDegrees(heading)); // rotate robot image
        g.drawImage(robotImage, -ROBOT_RADIUS, -ROBOT_RADIUS); // draw centered
		
		// Restore previous transform
        g.setTransform(old);
    }
}
