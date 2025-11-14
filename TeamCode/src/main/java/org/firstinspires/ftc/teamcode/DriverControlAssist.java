package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.dashboard.DashboardTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.RobotDrawing;

@TeleOp
@Config
public class DriverControlAssist extends LinearOpMode {
    private RobotCommon common;
    private Follower follower;
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    public static double ROBOT_SLOW = 0.5;
    public static double ROBOT_FAST = 1;
    public static double ROT_FAST = 0.5;
    public static double ROT_SLOW = 0.3;
    public static int LIFT_UP = 1;
    public static double SHOOTER_X = 1350;
    public static double SHOOTER_Y = 1400;
    public static double SHOOTER_START = 1550;
    private int headingOffset = 0;
    protected Pose halfShotPose = new Pose(27.5,27,Math.toRadians(45));
    protected Pose farShotPose = new Pose(-54.5,12.5,Math.toRadians(23.5));
    protected Pose midShotPose = new Pose(0,0,Math.toRadians(45));
    protected Pose closeShotPose = new Pose(42, -4, Math.toRadians(73));

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                follower.update();
                common.runAuton();
                sendTelemetry();
            }
        }
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        common = new RobotCommon();
        common.initialize(hardwareMap);
        follower = (Follower) blackboard.get("follower");
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
        }
        follower.update();
        follower.startTeleOpDrive(true);
        if (blackboard.containsKey("headingOffset")) {
            headingOffset = (int)blackboard.get("headingOffset");
        }
        sendTelemetry();
    }

    private void sendTelemetry() {
        common.addPedroPathingTelemetry(telemetry, dashboardTelemetry, follower);
        RobotDrawing.draw(dashboardTelemetry.getCurrentPacket(), follower);
        telemetry.addData("headingOffset", headingOffset);
        telemetry.addData("isTeleopDrive", follower.isTeleopDrive());
        common.sendTelemetry(telemetry);
    }

    private void controls() {
        if (gamepad2.a) {
            common.setIntakeDirection(RobotCommon.ShaftDirection.IN);
        } else if (gamepad2.b) {
            common.setIntakeDirection(RobotCommon.ShaftDirection.OUT);
        } else {
            common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
        }
        if (gamepad1.dpad_up) {
            common.setLiftTargetPosition(LIFT_UP);
        }
        if (gamepad2.y) {
            common.setShooterTarget(SHOOTER_Y);
        } else if (gamepad2.x) {
            common.setShooterTarget(SHOOTER_X);
        } else if (gamepad2.back) {
            common.setShooterTarget(SHOOTER_START);
        } else if (gamepad2.guide) {
            common.setShooterTarget(0);
        }

        common.setJamFix(gamepad2.dpad_down);

        if (gamepad2.right_bumper) {
            common.setFeederDirection(RobotCommon.ShaftDirection.IN);
        } else if (gamepad2.left_bumper) {
            common.setFeederDirection(RobotCommon.ShaftDirection.OUT);
        } else {
            common.setFeederDirection(RobotCommon.ShaftDirection.STOP);
        }

        if (gamepad1.x) {
            if (follower.isTeleopDrive()) {
                goToPose(halfShotPose);
            }
        } else if (gamepad1.y) {
            if (follower.isTeleopDrive()) {
                goToPose(midShotPose);
            }
        } else if (gamepad1.back) {
            if (follower.isTeleopDrive()) {
                goToPose(farShotPose);
            }
        } else if (gamepad1.right_bumper) {
            if (follower.isTeleopDrive()) {
                goToPose(closeShotPose);
            }
        } else {
            if (!follower.isTeleopDrive()) {
                follower.startTeleOpDrive(true);
            }
        }
        double speed = ROBOT_FAST;
        double rotSpeed = ROT_FAST;
        if (gamepad1.a) {
            speed = ROBOT_SLOW;
            rotSpeed = ROT_SLOW;
        }

        double x = square(-gamepad1.left_stick_y) * speed;
        double y = square(gamepad1.left_stick_x) * speed;
        if (gamepad1.guide)  {
            headingOffset = (int) Math.toDegrees(follower.getHeading());
            blackboard.put("headingOffset", headingOffset);
        }
        double rot = square(gamepad1.right_trigger-gamepad1.left_trigger) * rotSpeed;

        follower.setTeleOpDrive(x, -y, -rot, false, Math.toRadians(headingOffset));

    }

    private void goToPose(Pose target) {
        if (headingOffset < 0) {
            target = RobotCommon.mirror(target);
        }
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setLinearHeadingInterpolation(follower.getHeading(), target.getHeading())
                .build();
        follower.followPath(path);
    }

    public static double square(double amount) {
        return amount * Math.abs(amount);
    }

}