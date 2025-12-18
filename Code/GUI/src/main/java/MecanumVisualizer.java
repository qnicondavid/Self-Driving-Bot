import javafx.animation.AnimationTimer;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;

import java.util.ArrayList;

public class MecanumVisualizer extends Canvas {

    private int FL, FR, BL, BR;

    private double posX, posY;
    private double heading = 0.0;

    private static final double ROBOT_RADIUS = 15.0;
    private static final double MOTOR_SCALE = 0.2;
    private static final double OMEGA_SCALE = 0.02;
    private static final double MOVE_SCALE = 0.5;

    private final ArrayList<double[]> path = new ArrayList<>();

    public MecanumVisualizer(double width, double height) {
        super(width, height);
        resetToCenter();

        new AnimationTimer() {
            @Override
            public void handle(long now) {
                stepRobot();
                draw();
            }
        }.start();
    }

    public void updateMotors(int fl, int fr, int bl, int br) {
        this.FL = fl;
        this.FR = fr;
        this.BL = bl;
        this.BR = br;
    }

    private void resetToCenter() {
        posX = getWidth() / 2.0;
        posY = getHeight() / 2.0;
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
        if (path.size() > 2000) path.remove(0);
    }

    public void draw() {
        GraphicsContext g = getGraphicsContext2D();

        g.setFill(Color.web("#151515"));
        g.fillRect(0, 0, getWidth(), getHeight());

        g.setStroke(Color.CYAN);
        g.setLineWidth(2);
        for (int i = 0; i < path.size() - 1; i++) {
            double[] p1 = path.get(i);
            double[] p2 = path.get(i + 1);
            g.strokeLine(p1[0], p1[1], p2[0], p2[1]);
        }

        g.setFill(Color.ORANGE);
        g.fillOval(posX - ROBOT_RADIUS, posY - ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 2);

        double hx = posX + Math.cos(heading) * ROBOT_RADIUS * 1.5;
        double hy = posY - Math.sin(heading) * ROBOT_RADIUS * 1.5;
        g.setStroke(Color.RED);
        g.setLineWidth(2);
        g.strokeLine(posX, posY, hx, hy);
    }
}
