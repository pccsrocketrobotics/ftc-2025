package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import java.util.function.Supplier;

@Autonomous
public class TestAuton extends LinearOpMode {
    private Follower follower;
    public static Pose startingPose1;
    public static Pose endingPose1;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private RobotCommon common;

    @Override
    public void runOpMode() {

        follower.setMaxPower(1.00); //causes code to crash with NullPointerExeption error
        initialize();
        Pose wallStartPose = new Pose(0, 0, Math.toRadians(0));
        Pose gateStartPose = new Pose(198, 10, Math.toRadians(40));
        Pose wallShootingPose = new Pose(0, 0, Math.toRadians(20));
        Pose gateShootingPose = new Pose(61, 0, Math.toRadians(45));
        Pose gateEndPose = new Pose(160, 10, Math.toRadians(90));
        Pose wallEndPose = new Pose(30, 10, Math.toRadians(90));

        follower.setStartingPose(wallStartPose);

        PathChain wallShooting = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(wallStartPose.getX(),wallStartPose.getY()), new Pose(wallShootingPose.getX(), wallShootingPose.getY())))
                .setLinearHeadingInterpolation(wallStartPose.getHeading(),wallShootingPose.getHeading())
                //.addPath(new BezierLine(new Point(whitePose), new Point(whitePose)))
                //.setPathEndTimeoutConstraint(0)
                //STARTED FIRST PATH CALL IT NEXT TIME
                .build();

        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {
                common.run();
                common.odo.update();
                common.sendTelemetry(telemetry);
            }
        }
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common = new RobotCommon();
        common.initialize(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose1 == null ? new Pose(0, 0) : startingPose1);
        follower.update();
        common.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }
}






