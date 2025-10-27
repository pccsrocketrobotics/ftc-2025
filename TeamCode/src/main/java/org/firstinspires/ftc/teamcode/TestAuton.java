package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

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
    public void runOpMode () {

        follower.setMaxPower(1.00); //causes code to crash with NullPointerExeption error
        initialize();

        waitForStart();
        if(opModeIsActive()) {

            PathChain supplier = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(20,0))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, .7853, 0.8))
                    .build();


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
        follower.setStartingPose(startingPose1 == null ? new Pose(0,0) : startingPose1);
        follower.update();
        common.odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
    }






