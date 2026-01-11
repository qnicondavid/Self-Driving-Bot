import javafx.geometry.Pos;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.control.Label;
import javafx.scene.layout.VBox;
import javafx.scene.layout.StackPane;
import javafx.scene.text.Font;
import javafx.scene.layout.HBox;
import javafx.geometry.Insets;
import javafx.scene.layout.Priority;
import javafx.animation.AnimationTimer;


/**
 * BotDataPane displays the real-time data of the robot, including:
 * - IR sensor readings (analog and digital)
 * - Ultrasonic sensor distance
 * - Emergency stop status
 * - Motor PWM values (FL, FR, BL, BR)
 * - Control state (PID, reverse, maze)
 * 
 * All labels are overlaid on an image of the robot to visualize its status.
 */
public class BotDataPane extends VBox {

	// IR sensor labels
    public Label irLeftLabel = new Label("Digital: --\nAnalog: --");
    public Label irRightLabel = new Label("Digital: --\nAnalog: --");
	
	// Ultrasonic and emergency stop labels
    public Label ultrasonicLabel = new Label("Ultrasonic Distance: -- cm");
	public Label emergencyLabel = new Label("Emergency Stop: --");

	// Motor speed labels
    public Label motorFLLabel = new Label("FL:\n -- pwm");
    public Label motorFRLabel = new Label("FR:\n -- pwm");
    public Label motorBLLabel = new Label("BL:\n -- pwm");
    public Label motorBRLabel = new Label("BR:\n -- pwm");
	
	// Control state label (PID, Reverse, Maze)
	public final Label controlLabel = new Label("");
	
	// Robot controller instance
	public RobotController controller = new RobotController(() -> {});

	// Status label showing connection state
    public Label statusLabel = new Label("Status: Connecting...");
	
	// Container for ultrasonic and emergency labels
	public HBox ultrasonicBox = new HBox(40);
	
	// Robot image view and its container
    public ImageView robotView;
    public StackPane robotStack;
	

    /**
     * Constructs the BotDataPane with all robot info labels positioned over the robot image.
     * 
     * @param controller the RobotController used to get current robot state
     */
    public BotDataPane(RobotController controller) {
		this.controller = controller;
		
		// Main VBox styling
        this.setStyle("-fx-background-color: #151515;");
        this.setAlignment(Pos.TOP_CENTER);
        this.setSpacing(10);
		
		// Load robot image and add to a StackPane
        Image robotImage = new Image(getClass().getResourceAsStream("/robot.png"));
        robotView = new ImageView(robotImage);
        robotView.setPreserveRatio(true);
        robotView.setFitWidth(540);

        robotStack = new StackPane();
        robotStack.getChildren().add(robotView);
        robotStack.setAlignment(Pos.CENTER);

		// Position sensor and motor labels relative to the robot
		positionSpeedLabels();
		positionIRLabels();
		positionURLabels();
		positionControlLabels();
		
		// Style the status label		
		styleLabel(statusLabel);
		
		// Add components to main VBox
        this.getChildren().addAll(ultrasonicBox, robotStack, statusLabel);
    }
	
	/**
     * Positions the control state label (PID, Reverse, Maze) on the left of the robot image
     * and updates it continuously with an AnimationTimer.
     */
	public void positionControlLabels() {
		styleCLabel(controlLabel);
		positionLabelOnRobot(controlLabel, -285, 0);
		AnimationTimer timer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                controlLabel.setText(controller.pidText() + "\n" + controller.reverseText() + "\n" + controller.mazeText());
            }
        };
        timer.start();
    }
	
	/**
     * Positions the ultrasonic and emergency stop labels above the robot image in a horizontal box.
     */
	public void positionURLabels() {
		styleLabel(ultrasonicLabel);
		styleLabel(emergencyLabel);

		ultrasonicBox.setAlignment(Pos.CENTER);
		ultrasonicBox.setPadding(new Insets(10, 0, 0, 0));

		ultrasonicLabel.setMaxWidth(Double.MAX_VALUE);
		emergencyLabel.setMaxWidth(Double.MAX_VALUE);

		HBox.setHgrow(ultrasonicLabel, Priority.ALWAYS);
		HBox.setHgrow(emergencyLabel, Priority.ALWAYS);

		ultrasonicBox.getChildren().addAll(
			ultrasonicLabel,
			emergencyLabel
		);
	}


    /**
     * Positions the IR sensor labels on top of the robot image.
     */
	public void positionIRLabels() {
		styleIRLabels(irLeftLabel);
        positionLabelOnRobot(irLeftLabel, -175, -315);
		
		styleIRLabels(irRightLabel);
		positionLabelOnRobot(irRightLabel, 175, -315);
	}

    /**
     * Positions the motor PWM labels around the robot image to visualize motor speeds.
     */
	public void positionSpeedLabels() {
		styleSpeedLabels(motorFLLabel);
        positionLabelOnRobot(motorFLLabel, -320, -200);
		
		styleSpeedLabels(motorFRLabel);
        positionLabelOnRobot(motorFRLabel, 309, -200);
		
		styleSpeedLabels(motorBRLabel);
        positionLabelOnRobot(motorBRLabel, 309, 228);
		
		styleSpeedLabels(motorBLLabel);
        positionLabelOnRobot(motorBLLabel, -320, 228);
	}
	
	 // --- Styling methods ---
	 
    public void styleLabel(Label label) {
        label.setFont(Font.font("Consolas", 20));
		label.setStyle("""
			-fx-text-fill: #e97132;
			-fx-alignment: center;
			-fx-text-alignment: center;
			-fx-padding: 0;
		""");
		label.setWrapText(true);
    }
	
	public void styleCLabel(Label label) {
        label.setFont(Font.font("Consolas", 20));
		label.setStyle("""
			-fx-text-fill: #e97132;
			-fx-alignment: center;
			-fx-text-alignment: left;
			-fx-padding: 0;
		""");
		label.setWrapText(true);
    }
	
	public void styleIRLabels(Label label) {
        label.setFont(Font.font("Consolas", 20));
		label.setStyle("""
			-fx-text-fill: #0f9ed5;
			-fx-alignment: center;
			-fx-text-alignment: center;
			-fx-padding: 0;
		""");
		label.setWrapText(true);
    }
	
	public void styleSpeedLabels(Label label) {
        label.setFont(Font.font("Consolas", 20));
		label.setStyle("""
			-fx-text-fill: #0f9ed5;
			-fx-alignment: center;
			-fx-text-alignment: center;
			-fx-padding: 0;
		""");
		label.setWrapText(true);
    }

    /**
     * Adds a label to the robot StackPane and positions it relative to the robot center.
     *
     * @param label   the label to position
     * @param offsetX horizontal offset from robot center
     * @param offsetY vertical offset from robot center
     */
    public void positionLabelOnRobot(Label label, double offsetX, double offsetY) {
        if (!robotStack.getChildren().contains(label)) {
            robotStack.getChildren().add(label);
        }
        StackPane.setAlignment(label, Pos.CENTER);
        label.setTranslateX(offsetX);
        label.setTranslateY(offsetY);
    }
}
