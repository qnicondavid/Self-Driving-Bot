import javafx.animation.AnimationTimer;
import javafx.geometry.Pos;
import javafx.scene.layout.VBox;

public class RightPane extends VBox {

    private final MecanumVisualizer visualizer;

    public RightPane() {
        this.setStyle("-fx-background-color: #151515;");
        this.setAlignment(Pos.CENTER);

        visualizer = new MecanumVisualizer(800, 600);

        this.getChildren().add(visualizer);

        AnimationTimer timer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                visualizer.stepRobot();
                visualizer.draw();
            }
        };
        timer.start();
    }

    public void updateMotors(int fl, int fr, int bl, int br) {
        visualizer.updateMotors(fl, fr, bl, br);
    }
}
