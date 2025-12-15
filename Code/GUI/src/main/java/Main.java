import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

import java.io.InputStream;
import java.net.Socket;

public class Main extends Application {

    private Label irLeftAnalogLabel = new Label("IR Left (Analog): --");
    private Label irLeftDigitalLabel = new Label("IR Left (Digital): --");
    private Label irRightAnalogLabel = new Label("IR Right (Analog): --");
    private Label irRightDigitalLabel = new Label("IR Right (Digital): --");
    private Label ultrasonicLabel = new Label("Ultrasonic Distance (cm): --");
    private Label statusLabel = new Label("Status: Connecting...");

    @Override
    public void start(Stage stage) {
        GridPane grid = new GridPane();
        grid.setVgap(5);
        grid.setHgap(10);

        grid.add(irLeftAnalogLabel, 0, 0);
        grid.add(irLeftDigitalLabel, 0, 1);
        grid.add(irRightAnalogLabel, 0, 2);
        grid.add(irRightDigitalLabel, 0, 3);
        grid.add(ultrasonicLabel, 0, 4);
        grid.add(statusLabel, 0, 5);

        VBox root = new VBox(grid);
        root.setSpacing(10);

        Scene scene = new Scene(root, 400, 200);
        stage.setTitle("Self-Driving Bot");
        stage.setScene(scene);
        stage.show();

        new Thread(this::readSensorData).start();
    }

    private void readSensorData() {
        String ip = "192.168.4.1";
        int port = 80;

        try (Socket socket = new Socket(ip, port)) {
            InputStream in = socket.getInputStream();
            StringBuilder buffer = new StringBuilder();
            boolean headerSkipped = false;
            int ch;

            while ((ch = in.read()) != -1) {
                char c = (char) ch;

                if (!headerSkipped) {
                    buffer.append(c);
                    if (buffer.toString().endsWith("\r\n\r\n")) {
                        buffer.setLength(0);
                        headerSkipped = true;
                        Platform.runLater(() -> statusLabel.setText("Status: Connected"));
                    }
                    continue;
                }

                if (c == '\n') {
                    String line = buffer.toString().trim();
                    buffer.setLength(0);
                    if (!line.isEmpty()) {
                        String finalLine = line;
                        Platform.runLater(() -> updateSensorValue(finalLine));
                    }
                } else if (c != '\r') {
                    buffer.append(c);
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
            Platform.runLater(() -> statusLabel.setText("Connection error: " + e.getMessage()));
        }
    }

    private void updateSensorValue(String line) {
        if (line.startsWith("IR Left (Analog):")) {
            irLeftAnalogLabel.setText(line);
        } else if (line.startsWith("IR Left (Digital):")) {
            irLeftDigitalLabel.setText(line);
        } else if (line.startsWith("IR Right (Analog):")) {
            irRightAnalogLabel.setText(line);
        } else if (line.startsWith("IR Right (Digital):")) {
            irRightDigitalLabel.setText(line);
        } else if (line.startsWith("Ultrasonic Distance")) {
            ultrasonicLabel.setText(line);
        }
    }

    public static void main(String[] args) {
        launch(args);
    }
}
