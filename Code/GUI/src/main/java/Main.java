import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.layout.*;
import javafx.stage.Stage;
import javafx.scene.control.ScrollPane;
import javafx.geometry.Insets;
import javafx.scene.input.KeyEvent;
import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.control.ScrollBar;
import javafx.scene.layout.Region;
import javafx.geometry.Orientation;


/**
 * Main is the entry point of the Self-Driving Bot JavaFX application.
 *
 * It sets up the main UI including:
 * - Left panel with bot data
 * - Right panel with tabs for commands, parameters, and path display
 * - Top menu bar for tab navigation
 * - Sensor reading thread
 * - Keyboard controls for robot
 */
public class Main extends Application {

	// Constants for the divider between left and right panels
    private static final double DIVIDER_THICKNESS = 3;
    private static final String DIVIDER_COLOR = "#e97132";

    @Override
    public void start(Stage stage) {
		
		// Initialize robot controller
		RobotController controller = new RobotController(() -> {});

		// Left-side pane showing bot data
        BotDataPane leftPane = new BotDataPane(controller);
		
		 // Overlay mask for right panel
        Mask mask = new Mask();
		
		// Right-side visualizations
        PathDisplayPane pathDisplayPane = new PathDisplayPane(controller);
		ParametersPane parametersPane = new ParametersPane(pathDisplayPane, controller);
		CommandsPane commandsPane = new CommandsPane();

		// ScrollPanes for right panel content
		ScrollPane scrollPane = new ScrollPane(commandsPane);
		styleScrollPane(scrollPane); // apply custom styling
		ScrollPane scrollParPane = new ScrollPane(parametersPane);
		styleScrollPane(scrollParPane);

		// StackPane allows switching content dynamically
        StackPane rightContent = new StackPane(mask);

		// Vertical divider between left and right panes
        Region verticalDivider = new Region();
        verticalDivider.setMinWidth(DIVIDER_THICKNESS);
        verticalDivider.setMaxWidth(DIVIDER_THICKNESS);
        verticalDivider.setStyle("-fx-background-color: " + DIVIDER_COLOR);

		// GridPane layout: 3 columns (left, divider, right)
        GridPane center = new GridPane();

        ColumnConstraints leftCol = new ColumnConstraints();
        leftCol.setPercentWidth(50); // left pane takes 50% width
        leftCol.setHgrow(Priority.ALWAYS);

        ColumnConstraints dividerCol = new ColumnConstraints();
        dividerCol.setMinWidth(DIVIDER_THICKNESS);
        dividerCol.setMaxWidth(DIVIDER_THICKNESS);

        ColumnConstraints rightCol = new ColumnConstraints();
        rightCol.setPercentWidth(50); // right pane takes 50% width
        rightCol.setHgrow(Priority.ALWAYS);

        center.getColumnConstraints().addAll(leftCol, dividerCol, rightCol);

		// Add left pane, divider, and right content to grid
        center.add(leftPane, 0, 0);
        center.add(verticalDivider, 1, 0);
        center.add(rightContent, 2, 0);

        GridPane.setVgrow(leftPane, Priority.ALWAYS);
        GridPane.setVgrow(rightContent, Priority.ALWAYS);

		// Top menu bar for tab navigation
        MainMenu menuBar = new MainMenu(rightContent, scrollPane, scrollParPane, pathDisplayPane);

		// Root layout: vertical box with menu bar on top
        VBox root = new VBox(menuBar, center);
        VBox.setVgrow(center, Priority.ALWAYS);

		// Create scene and stage
        Scene scene = new Scene(root, 900, 500);
        stage.setTitle("Self-Driving Bot");
        stage.setScene(scene);
        stage.setMaximized(true);
		
		// Keyboard input handling
		scene.addEventHandler(KeyEvent.KEY_PRESSED, controller::handleKeyPressed);
		scene.addEventHandler(KeyEvent.KEY_RELEASED, controller::handleKeyReleased);
		
        stage.show();
		
		// Start sensor reading in a separate daemon thread
        SensorReader reader = new SensorReader(leftPane, pathDisplayPane);
        Thread sensorThread = new Thread(reader);
        sensorThread.setDaemon(true);
        sensorThread.start();
    }
	

    /**
     * Styles a ScrollPane for the application's dark theme.
     *
     * @param scrollPane the ScrollPane to style
     */
	public static void styleScrollPane(ScrollPane scrollPane) {
		scrollPane.setFitToWidth(true);
		scrollPane.setPannable(true);
		scrollPane.setVbarPolicy(ScrollPane.ScrollBarPolicy.AS_NEEDED);
		scrollPane.setHbarPolicy(ScrollPane.ScrollBarPolicy.NEVER);

		scrollPane.setPadding(Insets.EMPTY);
		scrollPane.setStyle(
			"-fx-background-color: transparent;" + 
			"-fx-background-insets: 0;" +
			"-fx-border-insets: 0;"
		);

		// Apply scrollbar styling after scene is available
		scrollPane.sceneProperty().addListener((obs, oldScene, newScene) -> {
			if (newScene != null) {
				Platform.runLater(() -> applyScrollBarStyle(scrollPane));
			}
		});
	}

	/**
     * Applies custom dark-themed styling to the scroll bars of a ScrollPane.
     * Hides increment/decrement buttons and styles the thumb.
     *
     * @param scrollPane the ScrollPane whose scrollbars will be styled
     */
	private static void applyScrollBarStyle(ScrollPane scrollPane) {
		for (Node n : scrollPane.lookupAll(".scroll-bar")) {
			if (n instanceof ScrollBar bar) {
				// Set width/height for vertical/horizontal scrollbars
				if (bar.getOrientation() == Orientation.VERTICAL) bar.setPrefWidth(10);
				else bar.setPrefHeight(1);
				
				// Hide increment/decrement buttons
				Node increment = bar.lookup(".increment-button");
				Node decrement = bar.lookup(".decrement-button");
				if (increment != null) increment.setVisible(false);
				if (decrement != null) decrement.setVisible(false);

				// Style scrollbar background
				bar.setStyle("-fx-background-color: #1e1e1e; -fx-padding: 0;");

				 // Style scrollbar thumb
				bar.lookupAll(".thumb").forEach(thumb -> {
					if (thumb instanceof Region r) {
						r.setStyle(
							"-fx-background-color: #888888;" +
							"-fx-background-radius: 3;" +
							"-fx-background-insets: 0;"
						);
						// Hover effect for the thumb
						r.setOnMouseEntered(e -> r.setStyle(
							"-fx-background-color: #aaaaaa;" +
							"-fx-background-radius: 3;" +
							"-fx-background-insets: 0;"
						));
						r.setOnMouseExited(e -> r.setStyle(
							"-fx-background-color: #888888;" +
							"-fx-background-radius: 3;" +
							"-fx-background-insets: 0;"
						));
					}
				});
			}
		}
	}

    public static void main(String[] args) {
        launch(args);
    }
}
