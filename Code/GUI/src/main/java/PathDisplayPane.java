import javafx.animation.AnimationTimer;
import javafx.geometry.Insets;
import javafx.scene.control.Label;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;

public class PathDisplayPane extends Pane {

    private final MecanumVisualizer visualizer;

    public PathDisplayPane(RobotController controller) {
        this.setStyle("-fx-background-color: #151515;");

        visualizer = new MecanumVisualizer(720, 780);

        visualizer.setLayoutX(0);
        visualizer.setLayoutY(0);

        this.getChildren().addAll(visualizer);

        AnimationTimer timer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                visualizer.stepRobot();
                visualizer.draw();
            }
        };
        timer.start();
    }
	
	public void setMotorScale(double motorScale) {
		visualizer.setMotorScale(motorScale);
	}
	
	public void setOmegaScale(double omegaScale) {
		visualizer.setOmegaScale(omegaScale);
	}
		
	public void setMoveScale(double moveScale) {
		visualizer.setMoveScale(moveScale);
	}
	
    public void updateMotors(int fl, int fr, int bl, int br) {
        visualizer.updateMotors(fl, fr, bl, br);
    }
}
