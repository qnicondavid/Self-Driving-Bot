import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;

import java.net.HttpURLConnection;
import java.net.URL;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

public class RobotController {

    private boolean pidEnabled = false;
    private boolean reverseEnabled = false;

    private final Set<KeyCode> pressedKeys = new HashSet<>();

    private final Runnable onStateChanged;

    public RobotController(Runnable onStateChanged) {
        this.onStateChanged = onStateChanged;
    }

    public void handleKeyPressed(KeyEvent event) {
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

        if (!pidEnabled) updateMovement();
    }

    public void handleKeyReleased(KeyEvent event) {
        pressedKeys.remove(event.getCode());
        if (!pidEnabled) updateMovement();
    }

    public String pidText() {
        return "PID: " + (pidEnabled ? "ON" : "OFF");
    }

    public String reverseText() {
        return "Reverse: " + (reverseEnabled ? "ON" : "OFF");
    }

    private void togglePID() {
        pidEnabled = !pidEnabled;

        if (pidEnabled && reverseEnabled) {
            reverseEnabled = false;
            sendRequestAsync("/move/stop");
        }

        sendRequestAsync(pidEnabled ? "/pid/on" : "/pid/off");
        clearMovementKeys();
    }

    private void toggleReverse() {
        reverseEnabled = !reverseEnabled;

        if (reverseEnabled && pidEnabled) {
            pidEnabled = false;
            sendRequestAsync("/pid/off");
        }

        sendRequestAsync(reverseEnabled ? "/move/south" : "/move/stop");
        clearMovementKeys();
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

    private void clearMovementKeys() {
        pressedKeys.remove(KeyCode.W);
        pressedKeys.remove(KeyCode.A);
        pressedKeys.remove(KeyCode.S);
        pressedKeys.remove(KeyCode.D);
        pressedKeys.remove(KeyCode.Q);
        pressedKeys.remove(KeyCode.E);
    }

    private void sendValue(String endpoint, double value) {
        sendRequestAsync(endpoint + "?value=" + value);
    }

    public void sendRequestAsync(String path) {
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
