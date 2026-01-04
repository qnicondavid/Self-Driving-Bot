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



public class Main extends Application {

    private static final double DIVIDER_THICKNESS = 3;
    private static final String DIVIDER_COLOR = "#e97132";

    @Override
    public void start(Stage stage) {
		
		RobotController controller = new RobotController(() -> {});

        BotDataPane leftPane = new BotDataPane(controller);
        Mask mask = new Mask();
        PathDisplayPane pathDisplayPane = new PathDisplayPane(controller);
		ParametersPane parametersPane = new ParametersPane(pathDisplayPane, controller);
		CommandsPane commandsPane = new CommandsPane();

		ScrollPane scrollPane = new ScrollPane(commandsPane);
		styleScrollPane(scrollPane);
		
		ScrollPane scrollParPane = new ScrollPane(parametersPane);
		styleScrollPane(scrollParPane);

        StackPane rightContent = new StackPane(mask);

        Region verticalDivider = new Region();
        verticalDivider.setMinWidth(DIVIDER_THICKNESS);
        verticalDivider.setMaxWidth(DIVIDER_THICKNESS);
        verticalDivider.setStyle("-fx-background-color: " + DIVIDER_COLOR);

        GridPane center = new GridPane();

        ColumnConstraints leftCol = new ColumnConstraints();
        leftCol.setPercentWidth(50); 
        leftCol.setHgrow(Priority.ALWAYS);

        ColumnConstraints dividerCol = new ColumnConstraints();
        dividerCol.setMinWidth(DIVIDER_THICKNESS);
        dividerCol.setMaxWidth(DIVIDER_THICKNESS);

        ColumnConstraints rightCol = new ColumnConstraints();
        rightCol.setPercentWidth(50);
        rightCol.setHgrow(Priority.ALWAYS);

        center.getColumnConstraints().addAll(leftCol, dividerCol, rightCol);

        center.add(leftPane, 0, 0);
        center.add(verticalDivider, 1, 0);
        center.add(rightContent, 2, 0);

        GridPane.setVgrow(leftPane, Priority.ALWAYS);
        GridPane.setVgrow(rightContent, Priority.ALWAYS);

        MainMenu menuBar = new MainMenu(rightContent, scrollPane, scrollParPane, pathDisplayPane);

        VBox root = new VBox(menuBar, center);
        VBox.setVgrow(center, Priority.ALWAYS);

        Scene scene = new Scene(root, 900, 500);
        stage.setTitle("Self-Driving Bot");
        stage.setScene(scene);
        stage.setMaximized(true);
		scene.addEventHandler(KeyEvent.KEY_PRESSED, controller::handleKeyPressed);
		scene.addEventHandler(KeyEvent.KEY_RELEASED, controller::handleKeyReleased);
        stage.show();

        SensorReader reader = new SensorReader(leftPane, pathDisplayPane);
        Thread sensorThread = new Thread(reader);
        sensorThread.setDaemon(true);
        sensorThread.start();
    }
	

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

		scrollPane.sceneProperty().addListener((obs, oldScene, newScene) -> {
			if (newScene != null) {
				Platform.runLater(() -> applyScrollBarStyle(scrollPane));
			}
		});
	}

	private static void applyScrollBarStyle(ScrollPane scrollPane) {
		for (Node n : scrollPane.lookupAll(".scroll-bar")) {
			if (n instanceof ScrollBar bar) {
				if (bar.getOrientation() == Orientation.VERTICAL) bar.setPrefWidth(10);
				else bar.setPrefHeight(1);

				Node increment = bar.lookup(".increment-button");
				Node decrement = bar.lookup(".decrement-button");
				if (increment != null) increment.setVisible(false);
				if (decrement != null) decrement.setVisible(false);

				bar.setStyle("-fx-background-color: #1e1e1e; -fx-padding: 0;");

				bar.lookupAll(".thumb").forEach(thumb -> {
					if (thumb instanceof Region r) {
						r.setStyle(
							"-fx-background-color: #888888;" +
							"-fx-background-radius: 3;" +
							"-fx-background-insets: 0;"
						);

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
