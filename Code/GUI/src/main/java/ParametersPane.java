import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.*;
import javafx.scene.effect.DropShadow;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;

import java.util.function.Consumer;

public class ParametersPane extends VBox {
	
	public PathDisplayPane pathDisplayPane = new PathDisplayPane(null);
	RobotController controller = null;

    public ParametersPane(PathDisplayPane pathDisplayPane, RobotController controller) {
		this.pathDisplayPane = pathDisplayPane;
		this.controller = controller;
        setPadding(new Insets(10, 20, 10, 20));
        setSpacing(20);
        setStyle("-fx-background-color: #151515;");

        getChildren().add(createSection(
                "Speed Parameter",
                "Set and validate the robot’s movement speed.",
                new Parameter[]{
                        new Parameter("Base Speed (0-255 pwm)", ParameterType.INT, 0, 255, 120, v -> onParameterChanged("/pid/base", v))
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
                "Bang-Bang Parameters",
                "Tune the Bang-Bang control variables.",
                new Parameter[]{
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
		
	private void changeMotorScale(Number v) {
		pathDisplayPane.setMotorScale(v.doubleValue());
	}
	
	private void changeOmegaScale(Number v) {
		pathDisplayPane.setOmegaScale(v.doubleValue());
	}
	
	private void changeMoveScale(Number v) {
		pathDisplayPane.setMoveScale(v.doubleValue());
	}

    private VBox createSection(String titleText, String infoText, Parameter[] parameters) {
        VBox section = new VBox(12);
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

        for (Parameter param : parameters) {
            section.getChildren().add(parameterRow(param));
        }

        return section;
    }

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

        submitButton.setOnAction(e -> {
            try {
                Number parsedValue = parameter.parse(inputField.getText());

                if (!parameter.isInRange(parsedValue)) {
                    inputField.setStyle("-fx-background-color: #5a1f1f; -fx-text-fill: white;");
                    return;
                }

                parameter.setValue(parsedValue);
                currentValueLabel.setText("Current: " + parameter.value);
                inputField.clear();
                inputField.setStyle("""
                        -fx-background-color: #2a2a2a;
                        -fx-text-fill: white;
                        -fx-background-radius: 6;
                        """);

            } catch (Exception ex) {
                inputField.setStyle("-fx-background-color: #5a1f1f; -fx-text-fill: white;");
            }
        });

        row.getChildren().addAll(nameLabel, inputField, currentValueLabel, submitButton);
        return row;
    }

    private void onParameterChanged(String endpoint, Number value) {
        this.controller.sendRequestAsync(endpoint + "?value=" + value.doubleValue());
    }


    enum ParameterType {
        INT, DOUBLE
    }

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

        Number parse(String text) {
            return switch (type) {
                case INT -> Integer.parseInt(text);
                case DOUBLE -> Double.parseDouble(text);
            };
        }

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

        void setValue(Number v) {
            this.value = v;
            onChange.accept(v);
        }
    }
}
