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
    public static Pose startingPose = new Pose(0,0,Math.toRadians(0));
    public static Pose halfShotPose = new Pose(-30,-27,Math.toRadians(45));
    public static Pose ballPickingPose= new Pose(-45,-12,Math.toRadians(90));
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private RobotCommon common;

    @Override
    public void runOpMode() {
        initialize();

        PathChain halfShotPath = follower.pathBuilder()
                .addPath(new BezierLine(startingPose,halfShotPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(),halfShotPose.getHeading())
                .build();
        PathChain ballPickPath = follower.pathBuilder()
                .addPath(new BezierLine(halfShotPose,ballPickingPose))
                .setLinearHeadingInterpolation(halfShotPose.getHeading(),ballPickingPose.getHeading())
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
        follower.setStartingPose(startingPose);
        follower.update();
        common.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }
}






