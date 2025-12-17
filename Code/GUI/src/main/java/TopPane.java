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
    private final Label pidHint;
    private final Label infoLabel;

    private double kP = 100;
    private double kI = 0.05;
    private double kD = 0;
    private int baseSpeed = 80;
    private int maxSpeed = 120;

    private final Set<KeyCode> pressedKeys = new HashSet<>();

    public TopPane() {
        Label title = new Label("Bot Commands");
        title.setFont(Font.font("Consolas", 20));
        title.setStyle("-fx-text-fill: #e97132;");
        title.setMaxWidth(Double.MAX_VALUE);
        title.setStyle(title.getStyle() + "-fx-alignment: center;");

        VBox leftColumn = new VBox();
        leftColumn.setSpacing(6);

        Label label1 = new Label("Press 'letter':");
        label1.setFont(Font.font("Consolas", 20));
        label1.setStyle("-fx-text-fill: #e97132;");

        pidHint = new Label(getPidText());
        pidHint.setFont(Font.font("Consolas", 20));
        pidHint.setStyle("-fx-text-fill: #e97132;");

        leftColumn.getChildren().addAll(label1, pidHint);

        VBox rightColumn = new VBox();
        rightColumn.setSpacing(6);

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

    private String getInfoText() {
        return String.format(
                "Press 'number' and '+' or '-':\n" +
                        "1. kP = %.2f\n" +
                        "2. kI = %.3f\n" +
                        "3. kD = %.2f\n" +
                        "4. Base Speed = %d\n" +
                        "5. Max Speed = %d",
                kP, kI, kD, baseSpeed, maxSpeed
        );
    }

    private void handleKeyPressed(KeyEvent event) {
        KeyCode code = event.getCode();
        pressedKeys.add(code);

        if (code == KeyCode.P) {
            togglePID();
            return;
        }

        int number = 0;
        if (pressedKeys.contains(KeyCode.DIGIT1)) number = 1;
        else if (pressedKeys.contains(KeyCode.DIGIT2)) number = 2;
        else if (pressedKeys.contains(KeyCode.DIGIT3)) number = 3;
        else if (pressedKeys.contains(KeyCode.DIGIT4)) number = 4;
        else if (pressedKeys.contains(KeyCode.DIGIT5)) number = 5;

        if (number != 0) {
            if (pressedKeys.contains(KeyCode.PLUS) || pressedKeys.contains(KeyCode.EQUALS)) {
                processChange(number, true);
            } else if (pressedKeys.contains(KeyCode.MINUS)) {
                processChange(number, false);
            }
        }
    }

    private void handleKeyReleased(KeyEvent event) {
        pressedKeys.remove(event.getCode());
    }

    private void processChange(int number, boolean increase) {
        switch (number) {
            case 1 -> {
                kP += increase ? 10 : -10;
                sendValue("/pid/kp", kP);
            }
            case 2 -> {
                kI += increase ? 0.01 : -0.01;
                sendValue("/pid/ki", kI);
            }
            case 3 -> {
                kD += increase ? 1 : -1;
                sendValue("/pid/kd", kD);
            }
            case 4 -> {
                baseSpeed += increase ? 5 : -5;
                sendValue("/pid/base", baseSpeed);
            }
            case 5 -> {
                maxSpeed += increase ? 5 : -5;
                sendValue("/pid/speed", maxSpeed);
            }
        }
        infoLabel.setText(getInfoText());
    }

    private void togglePID() {
        pidEnabled = !pidEnabled;
        pidHint.setText(getPidText());
        sendRequestAsync(pidEnabled ? "/pid/on" : "/pid/off");
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
