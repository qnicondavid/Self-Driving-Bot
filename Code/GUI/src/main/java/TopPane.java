import javafx.application.Platform;
import javafx.scene.control.Label;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;

import java.net.HttpURLConnection;
import java.net.URL;
import java.util.HashSet;
import java.util.Set;

public class TopPane extends VBox {

    private boolean pidEnabled = false;
	private boolean reverseEnabled = false;
	
    private final Label pidHint;
    private final Label movementHint;
    private final Label infoLabel;
	private final Label reverseHint;


    private int baseSpeed = 80;
    private int maxSpeed = 120;
    private double kP = 100;
    private double kI = 0.05;
    private double kD = 0;
	
	private int dStop = 1;
	private int deltaD = 1;

    private final Set<KeyCode> pressedKeys = new HashSet<>();

    public TopPane() {
        Label title = new Label("Bot Commands");
        title.setFont(Font.font("Consolas", 20));
        title.setStyle("-fx-text-fill: #e97132; -fx-alignment: center;");
        title.setMaxWidth(Double.MAX_VALUE);

        VBox leftColumn = new VBox(6);
        VBox rightColumn = new VBox(6);

        Label label1 = new Label("Press 'key' to toggle:");
        label1.setFont(Font.font("Consolas", 20));
        label1.setStyle("-fx-text-fill: #e97132;");

        pidHint = new Label(getPidText());
        pidHint.setFont(Font.font("Consolas", 20));
        pidHint.setStyle("-fx-text-fill: #e97132;");
		
		reverseHint = new Label(getReverseText());
		reverseHint.setFont(Font.font("Consolas", 20));
		reverseHint.setStyle("-fx-text-fill: #e97132;");

        movementHint = new Label(getMovementText());
        movementHint.setFont(Font.font("Consolas", 20));
        movementHint.setStyle("-fx-text-fill: #e97132;");

        leftColumn.getChildren().addAll(label1, pidHint, reverseHint, movementHint);

        infoLabel = new Label(getInfoText());
        infoLabel.setFont(Font.font("Consolas", 20));
        infoLabel.setStyle("-fx-text-fill: #0f9ed5;");
        rightColumn.getChildren().add(infoLabel);

        HBox columns = new HBox(leftColumn, rightColumn);
        HBox.setHgrow(leftColumn, Priority.ALWAYS);
        HBox.setHgrow(rightColumn, Priority.ALWAYS);
        leftColumn.setStyle("-fx-alignment: top-left;");
        rightColumn.setStyle("-fx-alignment: top-right;");

        this.getChildren().addAll(title, columns);
        this.setSpacing(10);
        this.setStyle("-fx-background-color: #151515; -fx-padding: 10;");
        this.setFocusTraversable(true);

        this.addEventHandler(KeyEvent.KEY_PRESSED, this::handleKeyPressed);
        this.addEventHandler(KeyEvent.KEY_RELEASED, this::handleKeyReleased);

        Platform.runLater(this::requestFocus);
    }

    private String getPidText() {
        return "'P' - PID: " + (pidEnabled ? "ON" : "OFF");
    }
	
	private String getReverseText() {
		return "'R' - Reverse: " + (reverseEnabled ? "ON" : "OFF");
	}

    private String getMovementText() {
        return "Hold 'key' to perform:\n" +
               "'W/A/S/D': Move N/W/S/E\n" +
               "Combine keys for diagonals.\n" +
               "'Q/E': Rotate CCW/CW";
    }

	private String getInfoText() {
		return String.format(
            "Press 'number' and '+' or '-':\n" +
            "1. Base Speed = %d\n" +
            "2. Max Speed = %d\n" +
            "3. kP = %.2f\n" +
            "4. kI = %.3f\n" +
            "5. kD = %.2f\n" +
            "6. d = %d cm\n" +
            "7. Δd = %d cm",
            baseSpeed, maxSpeed, kP, kI, kD, dStop, deltaD
		);
	}



    private void handleKeyPressed(KeyEvent event) {
        KeyCode code = event.getCode();
        if (!pressedKeys.add(code)) return;

        if (code == KeyCode.P) {
            togglePID();
            return;
        }
		
		if (code == KeyCode.R) {
			toggleReverse();
			return;
		}

        int number = 0;
        if (pressedKeys.contains(KeyCode.DIGIT1)) number = 1;
        else if (pressedKeys.contains(KeyCode.DIGIT2)) number = 2;
        else if (pressedKeys.contains(KeyCode.DIGIT3)) number = 3;
        else if (pressedKeys.contains(KeyCode.DIGIT4)) number = 4;
        else if (pressedKeys.contains(KeyCode.DIGIT5)) number = 5;
		else if (pressedKeys.contains(KeyCode.DIGIT6)) number = 6; 
		else if (pressedKeys.contains(KeyCode.DIGIT7)) number = 7; 

        if (number != 0) {
            if (pressedKeys.contains(KeyCode.PLUS) || pressedKeys.contains(KeyCode.EQUALS)) {
                processChange(number, true);
            } else if (pressedKeys.contains(KeyCode.MINUS)) {
                processChange(number, false);
            }
            return;
        }

        if (!pidEnabled) updateMovement();
    }

    private void handleKeyReleased(KeyEvent event) {
        pressedKeys.remove(event.getCode());
        if (!pidEnabled) updateMovement();
    }

	private void togglePID() {
		pidEnabled = !pidEnabled;

		if (pidEnabled && reverseEnabled) {
			reverseEnabled = false;
			reverseHint.setText(getReverseText());
			sendRequestAsync("/move/stop");
		}

		pidHint.setText(getPidText());
		sendRequestAsync(pidEnabled ? "/pid/on" : "/pid/off");

		pressedKeys.remove(KeyCode.W);
		pressedKeys.remove(KeyCode.A);
		pressedKeys.remove(KeyCode.S);
		pressedKeys.remove(KeyCode.D);
		pressedKeys.remove(KeyCode.Q);
		pressedKeys.remove(KeyCode.E);
	}

	private void toggleReverse() {
		reverseEnabled = !reverseEnabled;

		if (reverseEnabled && pidEnabled) {
			pidEnabled = false;
			pidHint.setText(getPidText());
			sendRequestAsync("/pid/off");
		}

		reverseHint.setText(getReverseText());

		if (reverseEnabled) {
			sendRequestAsync("/move/south");
		} else {
			sendRequestAsync("/move/stop");
		}

		pressedKeys.remove(KeyCode.W);
		pressedKeys.remove(KeyCode.A);
		pressedKeys.remove(KeyCode.S);
		pressedKeys.remove(KeyCode.D);
		pressedKeys.remove(KeyCode.Q);
		pressedKeys.remove(KeyCode.E);
	}


	private void processChange(int number, boolean increase) {
		switch (number) {
			case 1 -> { baseSpeed += increase ? 5 : -5; sendValue("/pid/base", baseSpeed); }
			case 2 -> { maxSpeed += increase ? 5 : -5; sendValue("/pid/speed", maxSpeed); }
			case 3 -> { kP += increase ? 10 : -10; sendValue("/pid/kp", kP); }
			case 4 -> { kI += increase ? 0.01 : -0.01; sendValue("/pid/ki", kI); }
			case 5 -> { kD += increase ? 0.01 : -0.01; sendValue("/pid/kd", kD); }
			case 6 -> { dStop += increase ? 1 : -1; sendValue("/emergency/stop", dStop); }
			case 7 -> { deltaD += increase ? 1 : -1; sendValue("/emergency/delta", deltaD); }
		}

		infoLabel.setText(getInfoText());
	}


    private void updateMovement() {
		if (reverseEnabled) return;
        boolean w = pressedKeys.contains(KeyCode.W);
        boolean a = pressedKeys.contains(KeyCode.A);
        boolean s = pressedKeys.contains(KeyCode.S);
        boolean d = pressedKeys.contains(KeyCode.D);
        boolean q = pressedKeys.contains(KeyCode.Q);
        boolean e = pressedKeys.contains(KeyCode.E);

        if (q && !e) { sendRequestAsync("/move/ccw"); return; }
        if (e && !q) { sendRequestAsync("/move/cw"); return; }

        if (w && a) { sendRequestAsync("/move/nw"); return; }
        if (w && d) { sendRequestAsync("/move/ne"); return; }
        if (s && a) { sendRequestAsync("/move/sw"); return; }
        if (s && d) { sendRequestAsync("/move/se"); return; }

        if (w) { sendRequestAsync("/move/north"); return; }
        if (s) { sendRequestAsync("/move/south"); return; }
        if (a) { sendRequestAsync("/move/west"); return; }
        if (d) { sendRequestAsync("/move/east"); return; }

        sendRequestAsync("/move/stop");
    }

    private void sendValue(String endpoint, double value) {
        sendRequestAsync(endpoint + "?value=" + value);
    }

    private void sendRequestAsync(String path) {
        new Thread(() -> {
            try {
                URL url = new URL("http://192.168.4.1" + path);
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("GET");
                conn.setConnectTimeout(500);
                conn.setReadTimeout(500);
                conn.getInputStream().close();
            } catch (Exception e) {
                System.err.println("Request failed: " + e.getMessage());
            }
        }).start();
    }
}
