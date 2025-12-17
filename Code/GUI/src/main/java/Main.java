import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

public class Main extends Application {

    @Override
    public void start(Stage stage) {

        TopPane topPane = new TopPane();
        BottomPane bottomPane = new BottomPane();
        RightPane rightPane = new RightPane();

        VBox leftPane = new VBox(topPane, bottomPane);
        VBox.setVgrow(topPane, Priority.ALWAYS);
        VBox.setVgrow(bottomPane, Priority.ALWAYS);

        topPane.prefHeightProperty().bind(leftPane.heightProperty().multiply(0.35));
        bottomPane.prefHeightProperty().bind(leftPane.heightProperty().multiply(0.65));

        HBox root = new HBox(leftPane, rightPane);
        HBox.setHgrow(leftPane, Priority.ALWAYS);
        HBox.setHgrow(rightPane, Priority.ALWAYS);

        leftPane.prefWidthProperty().bind(root.widthProperty().multiply(0.35));
        rightPane.prefWidthProperty().bind(root.widthProperty().multiply(0.65));

        Scene scene = new Scene(root, 800, 400);
        stage.setTitle("Self-Driving Bot");
        stage.setMaximized(true);
        stage.setScene(scene);
        stage.show();

		SensorReader reader = new SensorReader(bottomPane, rightPane);
		Thread sensorThread = new Thread(reader);
		sensorThread.setDaemon(true);
		sensorThread.start();
    }

    public static void main(String[] args) {
        launch(args);
    }
}
