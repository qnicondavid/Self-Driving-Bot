import javafx.application.Platform;
import java.io.InputStream;
import java.net.Socket;

public class SensorReader {

    private final BottomPane ui;

    public SensorReader(BottomPane ui) {
        this.ui = ui;
    }

    public void readSensorData() {
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
                        Platform.runLater(() ->
                            ui.statusLabel.setText("Status: Connected")
                        );
                    }
                    continue;
                }

                if (c == '\n') {
                    String line = buffer.toString().trim();
                    buffer.setLength(0);
                    if (!line.isEmpty()) {
                        Platform.runLater(() -> updateSensorValue(line));
                    }
                } else if (c != '\r') {
                    buffer.append(c);
                }
            }

        } catch (Exception e) {
            Platform.runLater(() ->
                ui.statusLabel.setText("Connection error: " + e.getMessage())
            );
        }
    }

	private void updateSensorValue(String line) {

    if (line.startsWith("IR Left (Analog):")) {
        ui.irLeftAnalogLabel.setText(line);

    } else if (line.startsWith("IR Left (Digital):")) {
        ui.irLeftDigitalLabel.setText(line);

    } else if (line.startsWith("IR Right (Analog):")) {
        ui.irRightAnalogLabel.setText(line);

    } else if (line.startsWith("IR Right (Digital):")) {
        ui.irRightDigitalLabel.setText(line);

    } else if (line.startsWith("Ultrasonic Distance")) {
        ui.ultrasonicLabel.setText(line);

    // 🔹 MOTORS (signed speeds)
    } else if (line.startsWith("Front Left:")) {
        ui.motorFLLabel.setText(line);

    } else if (line.startsWith("Front Right:")) {
        ui.motorFRLabel.setText(line);

    } else if (line.startsWith("Back Left:")) {
        ui.motorBLLabel.setText(line);

    } else if (line.startsWith("Back Right:")) {
        ui.motorBRLabel.setText(line);
    }
	}

}
