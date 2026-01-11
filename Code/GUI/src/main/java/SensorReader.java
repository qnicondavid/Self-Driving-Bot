import javafx.application.Platform;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.Socket;

	/**
	* Reads sensor data from the robot over a TCP socket and updates the UI.
	* Handles IR sensors (analog and digital), ultrasonic distance, motor PWM values,
	* and emergency stop status. Continuously maintains a connection and updates
	* JavaFX UI components on the Application Thread.
	*/
public class SensorReader implements Runnable {

    private static final String IP = "192.168.4.1";
    private static final int PORT = 80;

    private final BotDataPane botData;
    private final PathDisplayPane pathDisplay;

    private int fl, fr, bl, br;
	
	private String irLeftAnalog = "--";
	private String irLeftDigital = "--";
	private String irRightAnalog = "--";
	private String irRightDigital = "--";

	/**
     * Creates a SensorReader that updates the given UI components.
     *
     * @param botData the pane showing the robot's sensor data
     * @param pathDisplay the pane showing the robot's path and motor data
     */
    public SensorReader(BotDataPane botData, PathDisplayPane pathDisplay) {
        this.botData = botData;
        this.pathDisplay = pathDisplay;
    }
	
	/**
     * Connects to the robot and continuously reads sensor data.
     * Updates the UI asynchronously. If the connection fails,
     * waits 1 second before trying again.
     */
    @Override
    public void run() {
        while (true) {
            try (Socket socket = new Socket(IP, PORT)) {

                Platform.runLater(() ->
                        botData.statusLabel.setText("Status: Connected")
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
                        botData.statusLabel.setText("Status: Disconnected")
                );

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException ignored) {}
            }
        }
    }


    /**
     * Processes a line of sensor data and updates the UI if necessary.
     *
     * @param line the line received from the robot
     */
    private void handleLine(String line) {
        if (line.isEmpty() || line.startsWith("-")) return;

        Platform.runLater(() -> updateUI(line));
    }
	
	/**
     * Updates UI components based on the sensor data line.
     *
     * @param line the sensor data line
     */
    private void updateUI(String line) {

		if (line.startsWith("IR Left (Analog):")) {
			irLeftAnalog = line.split(":")[1].trim();
			botData.irLeftLabel.setText("Digital: " + irLeftDigital + "\nAnalog: " + irLeftAnalog);

		} else if (line.startsWith("IR Left (Digital):")) {
			irLeftDigital = line.split(":")[1].trim();
			botData.irLeftLabel.setText("Digital: " + irLeftDigital + "\nAnalog: " + irLeftAnalog);

		} else if (line.startsWith("IR Right (Analog):")) {
			irRightAnalog = line.split(":")[1].trim();
			botData.irRightLabel.setText("Digital: " + irRightDigital + "\nAnalog: " + irRightAnalog);

		} else if (line.startsWith("IR Right (Digital):")) {
			irRightDigital = line.split(":")[1].trim();
			botData.irRightLabel.setText("Digital: " + irRightDigital + "\nAnalog: " + irRightAnalog);

		} else if (line.startsWith("Ultrasonic Distance")) {
			botData.ultrasonicLabel.setText("Ultrasonic Distance: " + line.split(":")[1].trim() + " cm");

		} else if (line.startsWith("EMERGENCY STOP:")) {
			botData.emergencyLabel.setText("Emergency Stop: " + line.split(":")[1].trim());

		} else if (line.startsWith("Front Left:")) {
			fl = parseValue(line);
			botData.motorFLLabel.setText("FL:\n" + fl + " pwm");

		} else if (line.startsWith("Front Right:")) {
			fr = parseValue(line);
			botData.motorFRLabel.setText("FR:\n" + fr + " pwm");

		} else if (line.startsWith("Back Left:")) {
			bl = parseValue(line);
			botData.motorBLLabel.setText("BL:\n" + bl + " pwm");

		} else if (line.startsWith("Back Right:")) {
			br = parseValue(line);
			botData.motorBRLabel.setText("BR:\n" + br + " pwm");
			pathDisplay.updateMotors(fl, fr, bl, br);
		}
	}

	
	/**
     * Parses an integer value from a line of the form "Label: value".
     *
     * @param line the line containing the value
     * @return the parsed integer
     */
    private int parseValue(String line) {
        return Integer.parseInt(line.split(":")[1].trim());
    }
}
