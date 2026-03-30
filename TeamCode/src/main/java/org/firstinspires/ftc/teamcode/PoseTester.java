package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dashboard.DashboardTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.RobotDrawing;

@TeleOp
@Config
public class PoseTester extends LinearOpMode {
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    private Follower follower;
    public static Pose startingPose = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        RobotDrawing.setDashboardTelemetry(FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        sendTelemetry();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                follower.update();
                sendTelemetry();
            }
        }
    }

    private void sendTelemetry() {
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        RobotDrawing.draw(dashboardTelemetry.getCurrentPacket(), follower);
        telemetry.update();
    }
}
