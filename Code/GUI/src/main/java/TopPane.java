import javafx.application.Platform;
import javafx.scene.control.Label;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;

import java.net.HttpURLConnection;
import java.net.URL;

public class TopPane extends VBox {

    private boolean pidEnabled = false;
    private final Label pidHint;

    public TopPane() {

        Label title = new Label("Bot Commands");
        title.setFont(Font.font("Consolas", 20));
        title.setStyle("-fx-text-fill: #e97132;");

        pidHint = new Label(getPidText());
        pidHint.setFont(Font.font("Consolas", 20));
        pidHint.setStyle("-fx-text-fill: #e97132;");

        this.getChildren().addAll(title, pidHint);
        this.setSpacing(6);
        this.setStyle("-fx-background-color: #151515; -fx-padding: 10;");

        this.setFocusTraversable(true);

        this.setOnKeyPressed(event -> {
            if (event.getCode() == KeyCode.P) {
                togglePID();
            }
        });

        Platform.runLater(this::requestFocus);
    }

    private String getPidText() {
        return "'P': toggle PID (now " + (pidEnabled ? "ON" : "OFF") + ")";
    }

    private void togglePID() {
        pidEnabled = !pidEnabled;
        pidHint.setText(getPidText());

        String endpoint = pidEnabled ? "/pid/on" : "/pid/off";
        new Thread(() -> sendRequest(endpoint)).start();
    }

    private void sendRequest(String path) {
        try {
            URL url = new URL("http://192.168.4.1" + path);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");
            conn.setConnectTimeout(500);
            conn.setReadTimeout(500);
            conn.getInputStream().close();
        } catch (Exception e) {
            System.err.println("PID request failed: " + e.getMessage());
        }
    }
}
