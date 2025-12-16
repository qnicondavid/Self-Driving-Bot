import javafx.scene.control.Label;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;

public class TopPane extends VBox {

    public TopPane() {
        Label topLabel = new Label("Bot Commands");
        topLabel.setFont(Font.font("Consolas", 20));
        topLabel.setStyle("-fx-text-fill: #e97132;");

        this.getChildren().add(topLabel);
        this.setStyle("-fx-background-color: #151515; -fx-padding: 10;");
        this.setSpacing(10);
    }
}
