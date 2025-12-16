import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;

public class BottomPane extends VBox {

    public Label irLeftAnalogLabel = new Label("IR Left (Analog): --");
    public Label irLeftDigitalLabel = new Label("IR Left (Digital): --");
    public Label irRightAnalogLabel = new Label("IR Right (Analog): --");
    public Label irRightDigitalLabel = new Label("IR Right (Digital): --");
    public Label ultrasonicLabel = new Label("Ultrasonic Distance (cm): --");

    public Label motorFLLabel = new Label("Front Left: --");
    public Label motorFRLabel = new Label("Front Right: --");
    public Label motorBLLabel = new Label("Back Left: --");
    public Label motorBRLabel = new Label("Back Right: --");

    public Label statusLabel = new Label("Status: Connecting...");

    public BottomPane() {

        Label bottomTitle = new Label("Bot Data");
        bottomTitle.setFont(Font.font("Consolas", 20));
        bottomTitle.setStyle("-fx-text-fill: #e97132;");

        Label[] labels = {
            statusLabel,
            ultrasonicLabel,
            irLeftAnalogLabel,
            irLeftDigitalLabel,
            irRightAnalogLabel,
            irRightDigitalLabel,
            motorFLLabel,
            motorFRLabel,
            motorBLLabel,
            motorBRLabel
        };

        for (Label l : labels) {
            l.setStyle("-fx-text-fill: #e97132;");
            l.setFont(Font.font("Consolas", 20));
        }

        GridPane sensorGrid = new GridPane();
        sensorGrid.setVgap(12);
        sensorGrid.setHgap(12);

        sensorGrid.add(statusLabel, 0, 0);
        sensorGrid.add(ultrasonicLabel, 0, 1);

        sensorGrid.add(irLeftAnalogLabel, 0, 2);
        sensorGrid.add(irLeftDigitalLabel, 0, 3);
        sensorGrid.add(irRightAnalogLabel, 0, 4);
        sensorGrid.add(irRightDigitalLabel, 0, 5);

        sensorGrid.add(motorFLLabel, 0, 6);
        sensorGrid.add(motorFRLabel, 0, 7);
        sensorGrid.add(motorBLLabel, 0, 8);
        sensorGrid.add(motorBRLabel, 0, 9);

        this.getChildren().addAll(bottomTitle, sensorGrid);
        this.setSpacing(10);
        this.setStyle("-fx-padding: 10; -fx-background-color: #151515;");
    }
}
