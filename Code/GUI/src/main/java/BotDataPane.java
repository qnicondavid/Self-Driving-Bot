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

public class BotDataPane extends VBox {

    public Label irLeftLabel = new Label("Digital: --\nAnalog: --");
    public Label irRightLabel = new Label("Digital: --\nAnalog: --");
    public Label ultrasonicLabel = new Label("Ultrasonic Distance: -- cm");
	public Label emergencyLabel = new Label("Emergency Stop: --");

    public Label motorFLLabel = new Label("FL:\n -- pwm");
    public Label motorFRLabel = new Label("FR:\n -- pwm");
    public Label motorBLLabel = new Label("BL:\n -- pwm");
    public Label motorBRLabel = new Label("BR:\n -- pwm");
	
	public final Label controlLabel = new Label("");
	
	public RobotController controller = new RobotController(() -> {});

    public Label statusLabel = new Label("Status: Connecting...");
	
	public HBox ultrasonicBox = new HBox(40);
	
    public ImageView robotView;
    public StackPane robotStack;
	

    public BotDataPane(RobotController controller) {
		this.controller = controller;
		
        this.setStyle("-fx-background-color: #151515;");
        this.setAlignment(Pos.TOP_CENTER);
        this.setSpacing(10);
		
        Image robotImage = new Image(getClass().getResourceAsStream("/robot.png"));
        robotView = new ImageView(robotImage);
        robotView.setPreserveRatio(true);
        robotView.setFitWidth(540);

        robotStack = new StackPane();
        robotStack.getChildren().add(robotView);
        robotStack.setAlignment(Pos.CENTER);

		positionSpeedLabels();
		
		positionIRLabels();
		
		positionURLabels();
		
		positionControlLabels();
					
		styleLabel(statusLabel);
		
        this.getChildren().addAll(ultrasonicBox, robotStack, statusLabel);
    }
	
	public void positionControlLabels() {
		styleCLabel(controlLabel);
		positionLabelOnRobot(controlLabel, -285, 0);
		AnimationTimer timer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                controlLabel.setText(controller.pidText() + "\n" + controller.reverseText());
            }
        };
        timer.start();
    }
	
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


	public void positionIRLabels() {
		styleIRLabels(irLeftLabel);
        positionLabelOnRobot(irLeftLabel, -175, -315);
		
		styleIRLabels(irRightLabel);
		positionLabelOnRobot(irRightLabel, 175, -315);
	}

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

    public void positionLabelOnRobot(Label label, double offsetX, double offsetY) {
        if (!robotStack.getChildren().contains(label)) {
            robotStack.getChildren().add(label);
        }
        StackPane.setAlignment(label, Pos.CENTER);
        label.setTranslateX(offsetX);
        label.setTranslateY(offsetY);
    }
}
