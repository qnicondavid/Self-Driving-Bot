import javafx.application.Platform;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.Socket;

public class SensorReader implements Runnable {

    private static final String IP = "192.168.4.1";
    private static final int PORT = 80;

    private final BottomPane bottom;
    private final RightPane right;

    private int fl, fr, bl, br;

    public SensorReader(BottomPane bottom, RightPane right) {
        this.bottom = bottom;
        this.right = right;
    }

    @Override
    public void run() {
        while (true) {
            try (Socket socket = new Socket(IP, PORT)) {

                Platform.runLater(() ->
                        bottom.statusLabel.setText("Status: Connected")
                );

                BufferedReader in = new BufferedReader(
                        new InputStreamReader(socket.getInputStream())
                );

                String line;
                while ((line = in.readLine()) != null && !line.isEmpty()) {}

                while ((line = in.readLine()) != null) {
                    handleLine(line.trim());
                }

            } catch (Exception e) {
                Platform.runLater(() ->
                        bottom.statusLabel.setText("Status: Disconnected")
                );

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException ignored) {}
            }
        }
    }

    private void handleLine(String line) {
        if (line.isEmpty() || line.startsWith("-")) return;

        Platform.runLater(() -> updateUI(line));
    }

    private void updateUI(String line) {

        if (line.startsWith("IR Left (Analog):")) {
            bottom.irLeftAnalogLabel.setText(line);

        } else if (line.startsWith("IR Left (Digital):")) {
            bottom.irLeftDigitalLabel.setText(line);

        } else if (line.startsWith("IR Right (Analog):")) {
            bottom.irRightAnalogLabel.setText(line);

        } else if (line.startsWith("IR Right (Digital):")) {
            bottom.irRightDigitalLabel.setText(line);

        } else if (line.startsWith("Ultrasonic Distance")) {
            bottom.ultrasonicLabel.setText(line);
			
		} else if (line.startsWith("EMERGENCY STOP:")) {
            bottom.emergencyLabel.setText(line);

        } else if (line.startsWith("Front Left:")) {
            fl = parseValue(line);
            bottom.motorFLLabel.setText(line);

        } else if (line.startsWith("Front Right:")) {
            fr = parseValue(line);
            bottom.motorFRLabel.setText(line);

        } else if (line.startsWith("Back Left:")) {
            bl = parseValue(line);
            bottom.motorBLLabel.setText(line);

        } else if (line.startsWith("Back Right:")) {
            br = parseValue(line);
            bottom.motorBRLabel.setText(line);

            right.updateMotors(fl, fr, bl, br);
        }
    }

    private int parseValue(String line) {
        return Integer.parseInt(line.split(":")[1].trim());
    }
}
