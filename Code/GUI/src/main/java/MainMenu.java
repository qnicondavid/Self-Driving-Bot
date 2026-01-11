import javafx.geometry.Insets;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.ToggleGroup;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import javafx.scene.control.ScrollPane;

/**
 * MainMenu is a horizontal menu bar that allows the user to switch between
 * different sections of the UI: Bot Commands, Parameters, and Path Display.
 *
 * It uses ToggleButtons grouped in a ToggleGroup so only one tab is active at a time,
 * and changes the right-side content pane when a tab is selected.
 */
public class MainMenu extends HBox {

    /**
     * Constructs the main menu bar.
     *
     * @param rightContent   The StackPane where the selected content pane is displayed
     * @param commandsPane   ScrollPane containing bot commands controls
     * @param parametersPane ScrollPane containing parameters controls
     * @param pathDisplayPane The PathDisplayPane showing the robot visualization
     */
    public MainMenu(StackPane rightContent, ScrollPane commandsPane, ScrollPane parametersPane, PathDisplayPane pathDisplayPane) {
		// Styling for the HBox menu: spacing between buttons, padding, background color, and bottom border
        setSpacing(10);
        setPadding(new Insets(8, 12, 8, 12));
        setStyle("""
            -fx-background-color: #151515;
            -fx-border-color: #e97132;
            -fx-border-width: 0 0 2 0;
        """);

		// ToggleGroup ensures only one tab is selected at a time
        ToggleGroup group = new ToggleGroup();

		// Create three menu tabs
        ToggleButton botCommands = createTab("Bot Commands");
		ToggleButton parameters = createTab("Parameters");
        ToggleButton pathDisplay = createTab("Path Display");

		// Add buttons to the toggle group
        botCommands.setToggleGroup(group);
		parameters.setToggleGroup(group);
        pathDisplay.setToggleGroup(group);

		// Default selection is Bot Commands
        botCommands.setSelected(true);
        rightContent.getChildren().setAll(commandsPane);

		// Switch content in rightContent when a tab is clicked
        botCommands.setOnAction(e ->
                rightContent.getChildren().setAll(commandsPane)
        );
		
		parameters.setOnAction(e ->
                rightContent.getChildren().setAll(parametersPane)
        );

        pathDisplay.setOnAction(e ->
                rightContent.getChildren().setAll(pathDisplayPane)
        );

		// Add the tabs to the HBox menu
        getChildren().addAll(botCommands, parameters, pathDisplay);
    }

	/**
     * Creates a styled ToggleButton representing a menu tab.
     * 
     * Changes style when selected or deselected.
     *
     * @param text The text to display on the tab
     * @return a configured ToggleButton
     */
    private ToggleButton createTab(String text) {
        ToggleButton btn = new ToggleButton(text);
		
		// Default style: transparent background, white text, bold Consolas font
        btn.setStyle("""
            -fx-background-color: transparent;
            -fx-text-fill: white;
			-fx-font-family: "Consolas";
            -fx-font-size: 18px;
            -fx-font-weight: bold;
        """);

		// Update style when selected/deselected
        btn.selectedProperty().addListener((obs, oldVal, selected) -> {
            if (selected) {
				// Highlight selected tab with orange background
                btn.setStyle("""
                    -fx-background-color: #e97132;
                    -fx-text-fill: white;
					-fx-font-family: "Consolas";
                    -fx-font-size: 18px;
                    -fx-font-weight: bold;
                """);
            } else {
				// Reset style for unselected tab
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
