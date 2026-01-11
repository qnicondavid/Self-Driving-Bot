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

/**
 * Mask is a simple JavaFX VBox container that can act as an overlay/placeholder.
 *
 * It sets up spacing, background color, padding, and allows the pane to receive
 * keyboard focus.
 */
public class Mask extends VBox {
	
	/**
     * Constructs a Mask pane with default styling.
     * - Spacing of 10 between child elements
     * - Dark background color (#151515)
     * - Padding of 10
     * - Focus traversable, so it can receive key events
     */
	public Mask() {
		this.setSpacing(10); // vertical spacing between children
		this.setStyle("-fx-background-color: #151515; -fx-padding: 10;"); // styling
		this.setFocusTraversable(true); // allow pane to gain focus for key events
	}
}
