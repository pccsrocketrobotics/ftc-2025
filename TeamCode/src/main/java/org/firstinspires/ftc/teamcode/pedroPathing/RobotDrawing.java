package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;

/**
 * This is the Drawing class. It handles the drawing of stuff on FTC Dashboard, like the robot.
 *
 * @author Logan Nash
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/22/2024
 */
public class RobotDrawing {
    public static final double ROBOT_RADIUS = 9;
    private static Telemetry dashboardTelemetry;

    /**
     * Set the instance of the FTC Dashboard telemetry
     * @param telemetry
     */
    public static void setDashboardTelemetry(Telemetry telemetry) {
        dashboardTelemetry = telemetry;
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashboardDrawingHandler class is not needed.
     *
     * @param follower
     */
    public static void draw(TelemetryPacket packet, Follower follower) {
        drawField(packet);
        if (follower.getCurrentPath() != null) {
            drawPath(packet, follower.getCurrentPath(), "#3F51B5");
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(packet, new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), "#3F51B5");
        }
        drawPoseHistory(packet, follower.getPoseHistory(), "#4CAF50");
        drawRobot(packet, follower.getPose(), "#4CAF50");
    }

    public static void drawField(TelemetryPacket packet) {
        Canvas field = packet.fieldOverlay();
        field.drawImage("/images/decode-flipped.webp", 0, 0, 144, 144);
        field.drawGrid(0, 0, 144, 144, 7, 7);
    }

    /**
     * This adds instructions to the current packet to draw a robot at a specified Pose with a specified
     * color.
     *
     * @param pose the Pose to draw the robot at
     * @param color the color to draw the robot with
     */
    public static void drawRobot(TelemetryPacket packet, Pose pose, String color) {
        packet.fieldOverlay().setStroke(color);
        RobotDrawing.drawRobotOnCanvas(packet.fieldOverlay(), pose.copy());
    }
    /**
     * This adds instructions to the current packet to draw a Path with a specified color. If no
     * packet exists, then a new one is created.
     *
     * @param path the Path to draw
     * @param color the color to draw the Path with
     */
    public static void drawPath(TelemetryPacket packet, Path path, String color) {
        packet.fieldOverlay().setStroke(color);
        RobotDrawing.drawPath(packet.fieldOverlay(), path.getPanelsDrawingPoints());
    }
    /**
     * This adds instructions to the current packet to draw all the Paths in a PathChain with a
     * specified color. If no packet exists, then a new one is created.
     *
     * @param pathChain the PathChain to draw
     * @param color the color to draw the PathChain with
     */
    public static void drawPath(TelemetryPacket packet, PathChain pathChain, String color) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(packet, pathChain.getPath(i), color);
        }
    }
    /**
     * This adds instructions to the current packet to draw the pose history of the robot. If no
     * packet exists, then a new one is created.
     *
     * @param poseTracker the DashboardPoseTracker to get the pose history from
     * @param color the color to draw the pose history with
     */
    public static void drawPoseHistory(TelemetryPacket packet, PoseHistory poseTracker, String color) {
        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().strokePolyline(poseTracker.getXPositionsArray(), poseTracker.getYPositionsArray());
    }

    /**
     * This draws a robot on the Dashboard at a specified Pose. This is more useful for drawing the
     * actual robot, since the Pose contains the direction the robot is facing as well as its position.
     *
     * @param c the Canvas on the Dashboard on which this will draw at
     * @param t the Pose to draw at
     */
    public static void drawRobotOnCanvas(Canvas c, Pose t) {
        if (t == null || Double.isNaN(t.getX()) || Double.isNaN(t.getY()) || Double.isNaN(t.getHeading())) {
            return;
        }

        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);
        Vector v = t.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = t.getX() + v.getXComponent() / 2, y1 = t.getY() + v.getYComponent() / 2;
        double x2 = t.getX() + v.getXComponent(), y2 = t.getY() + v.getYComponent();
        c.strokeLine(x1, y1, x2, y2);
    }

    /**
     * This draws a Path on the Dashboard from a specified Array of Points.
     *
     * @param c the Canvas on the Dashboard on which this will draw
     * @param points the Points to draw
     */
    public static void drawPath(Canvas c, double[][] points) {
        c.strokePolyline(points[0], points[1]);
    }
}