import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.Label;
import javafx.scene.effect.DropShadow;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;

public class CommandsPane extends VBox {

    public CommandsPane() {
        setPadding(new Insets(10, 20, 10, 20));
        setSpacing(20);
        setStyle("-fx-background-color: #151515;");

		getChildren().add(createSection(
			"Robot Movements",
			"Hold the respective key to move the robot. Combine keys for diagonal movement.",
			new String[][]{
					{"Forward / North", "W"},
					{"Backward / South", "S"},
					{"Left / West", "A"},
					{"Right / East", "D"},
					{"Rotate Clockwise", "Q"},
					{"Rotate Counterclockwise", "E"}
			}
		));
		
		getChildren().add(createSection(
			"Special Manoeuvres",
			"Press the respective key to activate/deactivate or perform the manoeuvre.",
			new String[][]{
					{"Toggled Reverse in a Straight Line​", "R"},
					{"Timed Reverse in a Straight Line​", "1"},
					{"Stepped Turn", "2"},
					{"Emergency PID (Stops at the first obstacle detected)", "3"},
					{"Park in a Box Forward", "4"},
					{"Three Point Turn", "5"}
			}
		));
		
		getChildren().add(createSection(
			"Line Following",
			"Press the respective key to activate or deactivate the algorithm.",
			new String[][]{
					{"PID", "P"}
			}
		));
		
		getChildren().add(createSection(
			"Maze Solving",
			"Press the respective key to activate or deactivate the algorithm.",
			new String[][]{
				{"Random Choices", "T"},
				{"Manual Choices", "Y"}
			}
		));
		
		getChildren().add(createSection(
			"Kidnapped Robot Problem",
			"Press the respective key to activate or deactivate the algorithm.",
			new String[][]{
				{"Algorithm A", "N"},
				{"Algorithm B", "M"}
			}
		));
		
    }

    private VBox createSection(String titleText, String infoText, String[][] commands) {
        VBox section = new VBox(10);
        section.setPadding(new Insets(12));
        section.setStyle("""
                -fx-background-color: #1f1f1f;
                -fx-background-radius: 10;
                """);
        section.setEffect(new DropShadow(8, Color.BLACK));

        Label sectionTitle = new Label(titleText);
        sectionTitle.setFont(Font.font("Consolas", FontWeight.BOLD, 20));
        sectionTitle.setStyle("-fx-text-fill: #0f9ed5;");

        Label sectionInfo = new Label(infoText);
        sectionInfo.setFont(Font.font("Consolas", 12));
        sectionInfo.setStyle("-fx-text-fill: #b0b0b0;");

        section.getChildren().addAll(sectionTitle, sectionInfo);

        for (String[] cmd : commands) {
            section.getChildren().add(commandRow(cmd[0], cmd[1]));
        }

        return section;
    }

    private HBox commandRow(String action, String key) {
        HBox row = new HBox(10);
        row.setAlignment(Pos.CENTER_LEFT);

        Label actionLabel = new Label(action);
        actionLabel.setStyle("-fx-text-fill: white;");
        HBox.setHgrow(actionLabel, Priority.ALWAYS);

        Label keyLabel = new Label(key);
        keyLabel.setAlignment(Pos.CENTER);
        keyLabel.setMinWidth(36);
        keyLabel.setStyle("""
                -fx-text-fill: white;
                -fx-font-weight: bold;
                -fx-background-color: #0f9ed5;
                -fx-background-radius: 6;
                -fx-padding: 4 12 4 12;
                """);

        keyLabel.setOnMouseEntered(e -> keyLabel.setStyle("""
                -fx-text-fill: white;
                -fx-font-weight: bold;
                -fx-background-color: #1abcff;
                -fx-background-radius: 6;
                -fx-padding: 4 12 4 12;
                """));
        keyLabel.setOnMouseExited(e -> keyLabel.setStyle("""
                -fx-text-fill: white;
                -fx-font-weight: bold;
                -fx-background-color: #0f9ed5;
                -fx-background-radius: 6;
                -fx-padding: 4 12 4 12;
                """));

        row.getChildren().addAll(actionLabel, keyLabel);
        return row;
    }
}
