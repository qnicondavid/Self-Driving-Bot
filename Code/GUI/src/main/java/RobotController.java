import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;

import java.net.HttpURLConnection;
import java.net.URL;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;


/**
 * RobotController handles keyboard input to control the robot.
 * It supports toggling PID, reverse, and maze modes, as well as
 * sending movement and action commands to the robot via HTTP requests.
 * Key states are tracked to allow simultaneous key presses.
 */
public class RobotController {
	
	// Flags for the current robot state
    private boolean pidEnabled = false;
    private boolean reverseEnabled = false;
	private boolean mazeEnabled = false;
	
	// Set of currently pressed keys to handle multiple simultaneous inputs
    private final Set<KeyCode> pressedKeys = new HashSet<>();
	
	// Runnable callback to notify UI or other components when state changes
    private final Runnable onStateChanged;
	
	
	/**
     * Creates a RobotController with a callback for state changes.
     *
     * @param onStateChanged called when robot state changes (optional)
     */
    public RobotController(Runnable onStateChanged) {
        this.onStateChanged = onStateChanged;
    }
	
	
	/**
     * Handles a key press event.
     * Toggles modes (PID, reverse, maze) and sends movement or action commands.
     *
     * @param event the key press event
     */
    public void handleKeyPressed(KeyEvent event) {
        KeyCode code = event.getCode();
		
		// Ignore if key was already pressed
        if (!pressedKeys.add(code)) return;
		
		// Mode toggles
        if (code == KeyCode.P) {
            togglePID();
            return;
        }

        if (code == KeyCode.R) {
            toggleReverse();
            return;
        }
		
		if (code == KeyCode.T) {
            toggleMaze("/random");
            return;
        }
		
		if (code == KeyCode.Y) {
            toggleMaze("/manual");
            return;
        }
		
		// Specific action endpoints
		if (code == KeyCode.N) {
            sendRequestAsync("/kidnap/a");
            return;
        }
		
		if (code == KeyCode.M) {
            sendRequestAsync("/kidnap/b");
            return;
        }
		
		if (code == KeyCode.DIGIT1) {
            sendRequestAsync("/reverse/perform");
            return;
        }
		
		if (code == KeyCode.DIGIT2) {
            sendRequestAsync("/turn/perform");
            return;
        }
		
		if (code == KeyCode.DIGIT3) {
            sendRequestAsync("/emergency/pid");
            return;
        }
		
		if (code == KeyCode.DIGIT4) {
            sendRequestAsync("/park/perform");
            return;
        }
		
		if (code == KeyCode.DIGIT5) {
            sendRequestAsync("/threeturn/perform");
            return;
        }
		
		// If PID mode is off, update movement based on pressed keys
        if (!pidEnabled) updateMovement();
    }


    /**
     * Handles a key release event.
     * Removes the key from the pressed set and updates movement if needed.
     *
     * @param event the key release event
     */
    public void handleKeyReleased(KeyEvent event) {
        pressedKeys.remove(event.getCode());
        if (!pidEnabled) updateMovement();
    }
	
	
	/**
     * Returns a human-readable PID status string.
     *
     * @return "PID: ON" or "PID: OFF"
     */
    public String pidText() {
        return "PID: " + (pidEnabled ? "ON" : "OFF");
    }
	
	/**
     * Returns a human-readable reverse status string.
     *
     * @return "Reverse: ON" or "Reverse: OFF"
     */
    public String reverseText() {
        return "Reverse: " + (reverseEnabled ? "ON" : "OFF");
    }
	
	/**
     * Returns a human-readable maze mode status string.
     *
     * @return "Maze: ON" or "Maze: OFF"
     */
	public String mazeText() {
        return "Maze: " + (mazeEnabled ? "ON" : "OFF");
    }
	
	/**
     * Toggles PID mode.
     * If enabled, disables reverse and maze modes.
     * Sends corresponding HTTP requests to the robot.
     */
	private void togglePID() {
		pidEnabled = !pidEnabled;

		if (pidEnabled) {
			if (reverseEnabled) {
				reverseEnabled = false;
				sendRequestAsync("/move/stop");
			}
			if (mazeEnabled) {
				mazeEnabled = false;
				sendRequestAsync("/maze/off");
			}
		}

		sendRequestAsync(pidEnabled ? "/pid/on" : "/pid/off");
		clearMovementKeys();
	}
	
	
	/**
     * Toggles reverse mode.
     * If enabled, disables PID and maze modes.
     * Sends corresponding HTTP requests to the robot.
     */
	private void toggleReverse() {
		reverseEnabled = !reverseEnabled;

		if (reverseEnabled) {
			if (pidEnabled) {
				pidEnabled = false;
				sendRequestAsync("/pid/off");
			}
			if (mazeEnabled) {
				mazeEnabled = false;
				sendRequestAsync("/maze/off");
			}
		}

		sendRequestAsync(reverseEnabled ? "/move/south" : "/move/stop");
		clearMovementKeys();
	}
	
	/**
     * Toggles maze mode.
     * Accepts a path to determine which maze behavior to enable.
     * Disables PID and reverse modes if enabled.
     */
	private void toggleMaze(String s) {
		mazeEnabled = !mazeEnabled;

		if (mazeEnabled) {
			if (pidEnabled) {
				pidEnabled = false;
				sendRequestAsync("/pid/off");
			}
			if (reverseEnabled) {
				reverseEnabled = false;
				sendRequestAsync("/move/stop");
			}
		}
		sendRequestAsync(mazeEnabled ? "/maze/on" + s : "/maze/off");
		clearMovementKeys();
	}
	
	/**
     * Updates robot movement based on pressed keys.
     * Handles combinations for diagonal and rotational movement.
     * Stops movement if no relevant key is pressed.
     */
    private void updateMovement() {
        if (reverseEnabled) return;

        boolean w = pressedKeys.contains(KeyCode.W);
        boolean a = pressedKeys.contains(KeyCode.A);
        boolean s = pressedKeys.contains(KeyCode.S);
        boolean d = pressedKeys.contains(KeyCode.D);
        boolean q = pressedKeys.contains(KeyCode.Q);
        boolean e = pressedKeys.contains(KeyCode.E);
		
		// Rotation commands
        if (q && !e) { sendRequestAsync("/move/ccw"); return; }
        if (e && !q) { sendRequestAsync("/move/cw"); return; }
		
		// Diagonal movement
        if (w && a) { sendRequestAsync("/move/nw"); return; }
        if (w && d) { sendRequestAsync("/move/ne"); return; }
        if (s && a) { sendRequestAsync("/move/sw"); return; }
        if (s && d) { sendRequestAsync("/move/se"); return; }
		
		// Cardinal directions
        if (w) { sendRequestAsync("/move/north"); return; }
        if (s) { sendRequestAsync("/move/south"); return; }
        if (a) { sendRequestAsync("/move/west"); return; }
        if (d) { sendRequestAsync("/move/east"); return; }
		
		// Stop if no movement keys are pressed
        sendRequestAsync("/move/stop");
    }
	
	/**
     * Clears all movement-related keys from the pressed set.
     */
    private void clearMovementKeys() {
        pressedKeys.remove(KeyCode.W);
        pressedKeys.remove(KeyCode.A);
        pressedKeys.remove(KeyCode.S);
        pressedKeys.remove(KeyCode.D);
        pressedKeys.remove(KeyCode.Q);
        pressedKeys.remove(KeyCode.E);
    }

    /**
     * Sends a numeric value to the robot for a specific endpoint.
     *
     * @param endpoint the robot endpoint
     * @param value the value to send
     */
    private void sendValue(String endpoint, double value) {
        sendRequestAsync(endpoint + "?value=" + value);
    }
	
	/**
     * Sends an HTTP GET request asynchronously to the robot.
     *
     * @param path the request path (e.g., "/move/north")
     */
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
