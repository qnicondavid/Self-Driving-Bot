import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.*;
import javafx.scene.effect.DropShadow;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;

import java.util.function.Consumer;

/**
 * ParametersPane is a JavaFX VBox pane that allows the user to configure
 * various robot parameters, including speed, PID coefficients, timed reverse,
 * turn steps, emergency stop thresholds, and path display settings.
 *
 * Each parameter is displayed with a label, input field, current value display,
 * and an "Apply" button to send the change to the robot.
 */
public class ParametersPane extends VBox {
	
	// Path display pane for visualizing robot movement
	public PathDisplayPane pathDisplayPane = new PathDisplayPane(null);
	
	// Robot controller used to send commands when parameters change
	RobotController controller = null;
	
	/**
     * Constructs a ParametersPane with all parameter sections.
     *
     * @param pathDisplayPane reference to the PathDisplayPane for scale adjustments
     * @param controller      reference to RobotController for sending parameter updates
     */
    public ParametersPane(PathDisplayPane pathDisplayPane, RobotController controller) {
		this.pathDisplayPane = pathDisplayPane;
		this.controller = controller;
		
		// Styling and spacing for the main VBox
        setPadding(new Insets(10, 20, 10, 20));
        setSpacing(20);
        setStyle("-fx-background-color: #151515;");
		
		// Add sections for different parameter groups
        getChildren().add(createSection(
                "Speed Parameter",
                "Set and validate the robot’s movement speed.",
                new Parameter[]{
                        new Parameter("Base Speed (0-255 pwm)", ParameterType.INT, 0, 255, 120, v -> onParameterChanged("/pid/base", v))
                }
        ));
		
		getChildren().add(createSection(
                "PID Parameters",
                "Calibrate the PID control coefficients.",
                new Parameter[]{
                        new Parameter("Max Speed (0-255 pwm)", ParameterType.INT, 0, 255, 200, v -> onParameterChanged("/pid/speed", v)),
                        new Parameter("Kp", ParameterType.INT, 0, 500, 200.0, v -> onParameterChanged("/pid/kp", v)),
                        new Parameter("Ki", ParameterType.DOUBLE, 0.0, 1.0, 0.01, v -> onParameterChanged("/pid/ki", v)),
                        new Parameter("Kd", ParameterType.DOUBLE, 0.0, 1.0, 0.0, v -> onParameterChanged("/pid/kd", v))
                }
        ));
		
		getChildren().add(createSection(
                "Timed Reverse Parameter",
                "Set the duration of the timed reverse manoeuvre.",
                new Parameter[]{
                        new Parameter("Duration (ms)", ParameterType.INT, 0, 100000, 0, v -> onParameterChanged("/reverse/time", v))
                }
        ));
		
		getChildren().add(createSection(
                "Stepped Turn Parameters",
                "Set the behaviour for the stepped turn.",
                new Parameter[]{
						new Parameter("Number of steps", ParameterType.INT, 0, 100000, 37, v -> onParameterChanged("/turn/turnSteps", v)),
						new Parameter("Y Direction (1 = fw, -1 = bw)", ParameterType.INT, -1, 1, 1, v -> onParameterChanged("/turn/turnXDirection", v)),
						new Parameter("X Direction (1 = cw, -1 = ccw)", ParameterType.INT, -1, 1, 1, v -> onParameterChanged("/turn/turnYDirection", v)),
                        new Parameter("Movement Duration at Start and End (ms)", ParameterType.INT, 0, 100000, 1000, v -> onParameterChanged("/turn/turnTimeForwardStart", v)),
						new Parameter("X Movement Duration in a Step (ms)", ParameterType.INT, 0, 1000, 70, v -> onParameterChanged("/turn/turnTimeForwardCorner", v)),
						new Parameter("Y Duration in a Step (ms)", ParameterType.INT, 0, 1000, 70, v -> onParameterChanged("/turn/turnTimeRotationCorner", v))
                }
        ));

        getChildren().add(createSection(
                "Emergency Stop Parameters",
                "Adjust the emergency stop thresholds.",
                new Parameter[]{
                        new Parameter("d (cm)", ParameterType.INT, 0, 999, 1, v -> onParameterChanged("/emergency/stop", v)),
                        new Parameter("Δd (cm)", ParameterType.INT, 0, 999, 1, v -> onParameterChanged("/emergency/delta", v))
                }
        ));
		
		getChildren().add(createSection(
                "Park in a Box Forward Parameter",
                "Calibrate the parking of the robot.",
                new Parameter[]{
                        new Parameter("Forward Movement Duration at the End (ms)", ParameterType.INT, 0, 10000, 0, v -> onParameterChanged("/park/time", v))
                }
        ));
						
		getChildren().add(createSection(
                "Path Display Parameters",
                "Optimize the path display behaviour.",
                new Parameter[]{
					    new Parameter("Motor Scale", ParameterType.DOUBLE, 0.0, 100.0, 0.2, v -> changeMotorScale(v)),
						new Parameter("Omega Scale", ParameterType.DOUBLE, 0.0, 100.0, 0.02, v -> changeOmegaScale(v)),
						new Parameter("Move Scale", ParameterType.DOUBLE, 0.0, 100.0, 0.05, v -> changeMoveScale(v))
                }
        ));
    }
		
	/**
     * Updates the motor scale in the path display visualizer.
     */
	private void changeMotorScale(Number v) {
		pathDisplayPane.setMotorScale(v.doubleValue());
	}
	
	/**
     * Updates the omega (rotation) scale in the path display visualizer.
     */
	private void changeOmegaScale(Number v) {
		pathDisplayPane.setOmegaScale(v.doubleValue());
	}
	
	/**
     * Updates the movement scale in the path display visualizer.
     */
	private void changeMoveScale(Number v) {
		pathDisplayPane.setMoveScale(v.doubleValue());
	}

	/**
     * Creates a section containing a title, info label, and a set of parameters.
     *
     * @param titleText   the section title
     * @param infoText    descriptive info about the section
     * @param parameters  array of parameters to include
     * @return a VBox containing the section
     */
    private VBox createSection(String titleText, String infoText, Parameter[] parameters) {
        VBox section = new VBox(12);
        section.setPadding(new Insets(12));
        section.setStyle("""
                -fx-background-color: #1f1f1f;
                -fx-background-radius: 10;
                """);
        section.setEffect(new DropShadow(8, Color.BLACK));
		
		// Section title
        Label sectionTitle = new Label(titleText);
        sectionTitle.setFont(Font.font("Consolas", FontWeight.BOLD, 20));
        sectionTitle.setStyle("-fx-text-fill: #0f9ed5;");

		// Section info
        Label sectionInfo = new Label(infoText);
        sectionInfo.setFont(Font.font("Consolas", 12));
        sectionInfo.setStyle("-fx-text-fill: #b0b0b0;");

        section.getChildren().addAll(sectionTitle, sectionInfo);

		// Add each parameter as a row
        for (Parameter param : parameters) {
            section.getChildren().add(parameterRow(param));
        }

        return section;
    }

	/**
     * Creates a row for a single parameter with label, input, current value, and apply button.
     */
    private HBox parameterRow(Parameter parameter) {
        HBox row = new HBox(10);
        row.setAlignment(Pos.CENTER_LEFT);

        Label nameLabel = new Label(parameter.name);
        nameLabel.setStyle("-fx-text-fill: white;");
        nameLabel.setMinWidth(160);

        TextField inputField = new TextField();
        inputField.setPrefWidth(100);
        inputField.setStyle("""
                -fx-background-color: #2a2a2a;
                -fx-text-fill: white;
                -fx-background-radius: 6;
                """);

        Label currentValueLabel = new Label("Current: " + parameter.value);
        currentValueLabel.setStyle("-fx-text-fill: #b0b0b0;");
        currentValueLabel.setMinWidth(120);

        Button submitButton = new Button("Apply");
        submitButton.setStyle("""
                -fx-background-color: #0f9ed5;
                -fx-text-fill: white;
                -fx-font-weight: bold;
                -fx-background-radius: 6;
                """);
		// Handle applying the parameter value
        submitButton.setOnAction(e -> {
            try {
                Number parsedValue = parameter.parse(inputField.getText());
				// Validate range
                if (!parameter.isInRange(parsedValue)) {
                    inputField.setStyle("-fx-background-color: #5a1f1f; -fx-text-fill: white;");
                    return;
                }
				// Update parameter and reset input field styling
                parameter.setValue(parsedValue);
                currentValueLabel.setText("Current: " + parameter.value);
                inputField.clear();
                inputField.setStyle("""
                        -fx-background-color: #2a2a2a;
                        -fx-text-fill: white;
                        -fx-background-radius: 6;
                        """);

            } catch (Exception ex) {
				// Highlight input field in red on parse error
                inputField.setStyle("-fx-background-color: #5a1f1f; -fx-text-fill: white;");
            }
        });

        row.getChildren().addAll(nameLabel, inputField, currentValueLabel, submitButton);
        return row;
    }

    /**
     * Sends the updated parameter value to the robot via the controller.
     */
    private void onParameterChanged(String endpoint, Number value) {
        this.controller.sendRequestAsync(endpoint + "?value=" + value.doubleValue());
    }

    /**
     * Type of parameter: integer or double.
     */
    enum ParameterType {
        INT, DOUBLE
    }
	
	
	/**
     * Represents a single configurable parameter.
     * Contains name, type, range, current value, and a callback on change.
     */
    static class Parameter {
        String name;
        ParameterType type;
        Number min;
        Number max;
        Number value;
        Consumer<Number> onChange;

        Parameter(String name, ParameterType type, Number min, Number max, Number defaultValue, Consumer<Number> onChange) {
            this.name = name;
            this.type = type;
            this.min = min;
            this.max = max;
            this.value = defaultValue;
            this.onChange = onChange;
        }
		
		/** Parses a string input into the correct numeric type. */
        Number parse(String text) {
            return switch (type) {
                case INT -> Integer.parseInt(text);
                case DOUBLE -> Double.parseDouble(text);
            };
        }
		
		/** Checks if the value is within the allowed range. */
        boolean isInRange(Number v) {
            return switch (type) {
                case INT -> {
                    int val = v.intValue();
                    yield val >= min.intValue() && val <= max.intValue();
                }
                case DOUBLE -> {
                    double val = v.doubleValue();
                    yield val >= min.doubleValue() && val <= max.doubleValue();
                }
            };
        }
		
		/** Sets the parameter value and triggers the onChange callback. */
        void setValue(Number v) {
            this.value = v;
            onChange.accept(v);
        }
    }
}
