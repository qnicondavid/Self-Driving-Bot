import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;

public class Main extends Application {

    @Override
    public void start(Stage stage) {
        Pane root = new Pane();          
        Scene scene = new Scene(root, 400, 200);

        stage.setTitle("Self-Driving Bot");
        stage.setScene(scene);
        stage.setResizable(true);       
        stage.show();
    }

    public static void main(String[] args) {
        launch(args);
    }
}
