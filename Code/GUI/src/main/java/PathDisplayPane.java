import javafx.animation.AnimationTimer;
import javafx.geometry.Insets;
import javafx.scene.control.Label;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;

/**
 * PathDisplayPane is a JavaFX pane responsible for visualizing the robot's
 * movement and motor states using the MecanumVisualizer.
 * It runs an animation loop to continuously update the robot's position
 * and redraw the visualization.
 */
public class PathDisplayPane extends Pane {

	// Visualizer that handles drawing the robot and interpreting motor data
    private final MecanumVisualizer visualizer;


    /**
     * Creates a PathDisplayPane and initializes the visualizer and animation loop.
     *
     * @param controller the RobotController
     */
    public PathDisplayPane(RobotController controller) {
		
		// Set background color of the pane
        this.setStyle("-fx-background-color: #151515;");

		// Create visualizer with specified width and height
        visualizer = new MecanumVisualizer(720, 780);

        visualizer.setLayoutX(0);
        visualizer.setLayoutY(0);
		
		// Add visualizer to this pane
        this.getChildren().addAll(visualizer);
		
		// Animation loop that steps the robot simulation and redraws the visualizer
        AnimationTimer timer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                visualizer.stepRobot(); // update robot position/state
                visualizer.draw(); // redraw robot and environment
            }
        };
        timer.start();
    }
	
	/**
     * Sets the motor scale for the visualizer.
     *
     * @param motorScale scaling factor for motor visualization
     */
	public void setMotorScale(double motorScale) {
		visualizer.setMotorScale(motorScale);
	}
	
	
	/**
     * Sets the omega (rotational speed) scale for the visualizer.
     *
     * @param omegaScale scaling factor for rotation visualization
     */
	public void setOmegaScale(double omegaScale) {
		visualizer.setOmegaScale(omegaScale);
	}
	
	/**
     * Sets the movement scale for the visualizer.
     *
     * @param moveScale scaling factor for linear movement visualization
     */
	public void setMoveScale(double moveScale) {
		visualizer.setMoveScale(moveScale);
	}
	
	/**
     * Updates the motor values in the visualizer.
     *
     * @param fl front-left motor PWM
     * @param fr front-right motor PWM
     * @param bl back-left motor PWM
     * @param br back-right motor PWM
     */
    public void updateMotors(int fl, int fr, int bl, int br) {
        visualizer.updateMotors(fl, fr, bl, br);
    }
}
