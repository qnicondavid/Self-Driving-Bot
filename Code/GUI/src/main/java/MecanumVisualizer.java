import javafx.animation.AnimationTimer;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

import java.util.ArrayList;

public class MecanumVisualizer extends Canvas {

    private int FL, FR, BL, BR;

    private double posX, posY;
    private double heading = 0.0;

    private double ROBOT_RADIUS = 70.0;
    private double MOTOR_SCALE = 0.2;
    private double OMEGA_SCALE = 0.02;
    private double MOVE_SCALE = 0.05;

    private final ArrayList<double[]> path = new ArrayList<>();

    private final Image robotImage;

    public MecanumVisualizer(double width, double height) {
        super(width, height);
        positionRobot();

        robotImage = new Image(getClass().getResource("/robot.png").toExternalForm(),
                               ROBOT_RADIUS * 2, ROBOT_RADIUS * 2, true, true);

        new AnimationTimer() {
            @Override
            public void handle(long now) {
                stepRobot();
                draw();
            }
        }.start();
    }
	
	public void setMotorScale(double motorScale) {
		this.MOTOR_SCALE = motorScale;
	}
	
	public void setOmegaScale(double omegaScale) {
		this.OMEGA_SCALE = omegaScale;
	}
		
	public void setMoveScale(double moveScale) {
		this.MOVE_SCALE = moveScale;
	}

    public void updateMotors(int fl, int fr, int bl, int br) {
        this.FL = fl;
        this.FR = fr;
        this.BL = bl;
        this.BR = br;
    }

    private void positionRobot() {
        posX = 390;
        posY = 390;
        path.clear();
        path.add(new double[]{posX, posY});
    }

    public void stepRobot() {
        double vy = (FL + FR + BL + BR) / 4.0 * MOTOR_SCALE;
        double vx = (FL - FR - BL + BR) / 4.0 * MOTOR_SCALE;
        double omega = (-FL + FR - BL + BR) / 4.0 * MOTOR_SCALE;

        heading += omega * OMEGA_SCALE;

        double dx = vx * Math.cos(heading) - vy * Math.sin(heading);
        double dy = vx * Math.sin(heading) + vy * Math.cos(heading);

        posX += dx * MOVE_SCALE;
        posY -= dy * MOVE_SCALE;

        posX = Math.max(ROBOT_RADIUS, Math.min(getWidth() - ROBOT_RADIUS, posX));
        posY = Math.max(ROBOT_RADIUS, Math.min(getHeight() - ROBOT_RADIUS, posY));

        path.add(new double[]{posX, posY});
        if (path.size() > 3000) path.remove(0);
    }

    public void draw() {
        GraphicsContext g = getGraphicsContext2D();

        g.setFill(Color.web("#151515"));
        g.fillRect(0, 0, getWidth(), getHeight());

        g.setStroke(Color.web("#0f9ed5"));
        g.setLineWidth(2);
        for (int i = 0; i < path.size() - 1; i++) {
            double[] p1 = path.get(i);
            double[] p2 = path.get(i + 1);
            g.strokeLine(p1[0], p1[1], p2[0], p2[1]);
        }

        Affine old = g.getTransform();
        g.translate(posX, posY);
        g.rotate(-Math.toDegrees(heading));
        g.drawImage(robotImage, -ROBOT_RADIUS, -ROBOT_RADIUS);
        g.setTransform(old);
    }
}
