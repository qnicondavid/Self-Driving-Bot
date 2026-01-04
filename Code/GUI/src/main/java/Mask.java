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

public class Mask extends VBox {

	public Mask() {
		this.setSpacing(10);
		this.setStyle("-fx-background-color: #151515; -fx-padding: 10;");
		this.setFocusTraversable(true);
	}
}
