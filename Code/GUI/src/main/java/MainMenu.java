import javafx.geometry.Insets;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.ToggleGroup;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import javafx.scene.control.ScrollPane;


public class MainMenu extends HBox {

    public MainMenu(StackPane rightContent, ScrollPane commandsPane, ScrollPane parametersPane, PathDisplayPane pathDisplayPane) {

        setSpacing(10);
        setPadding(new Insets(8, 12, 8, 12));
        setStyle("""
            -fx-background-color: #151515;
            -fx-border-color: #e97132;
            -fx-border-width: 0 0 2 0;
        """);

        ToggleGroup group = new ToggleGroup();

        ToggleButton botCommands = createTab("Bot Commands");
		ToggleButton parameters = createTab("Parameters");
        ToggleButton pathDisplay = createTab("Path Display");

        botCommands.setToggleGroup(group);
		parameters.setToggleGroup(group);
        pathDisplay.setToggleGroup(group);

        botCommands.setSelected(true);
        rightContent.getChildren().setAll(commandsPane);

        botCommands.setOnAction(e ->
                rightContent.getChildren().setAll(commandsPane)
        );
		
		parameters.setOnAction(e ->
                rightContent.getChildren().setAll(parametersPane)
        );

        pathDisplay.setOnAction(e ->
                rightContent.getChildren().setAll(pathDisplayPane)
        );

        getChildren().addAll(botCommands, parameters, pathDisplay);
    }

    private ToggleButton createTab(String text) {
        ToggleButton btn = new ToggleButton(text);
        btn.setStyle("""
            -fx-background-color: transparent;
            -fx-text-fill: white;
			-fx-font-family: "Consolas";
            -fx-font-size: 18px;
            -fx-font-weight: bold;
        """);

        btn.selectedProperty().addListener((obs, oldVal, selected) -> {
            if (selected) {
                btn.setStyle("""
                    -fx-background-color: #e97132;
                    -fx-text-fill: white;
					-fx-font-family: "Consolas";
                    -fx-font-size: 18px;
                    -fx-font-weight: bold;
                """);
            } else {
                btn.setStyle("""
                    -fx-background-color: transparent;
                    -fx-text-fill: white;
					-fx-font-family: "Consolas";
                    -fx-font-size: 18px;
                    -fx-font-weight: bold;
                """);
            }
        });

        return btn;
    }
}
