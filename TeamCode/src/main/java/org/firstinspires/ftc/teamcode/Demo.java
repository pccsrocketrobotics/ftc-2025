package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dashboard.DashboardTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.RobotDrawing;

@TeleOp
@Config
public class Demo extends LinearOpMode {
    private RobotCommon common;
    private Follower follower;
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    public static double ROBOT_SLOW = 0.5;
    public static double ROT_SLOW = 0.3;
    public static double FULL_SHOOTING = 1000;
    public static double FULL_BOWLING = -1000;
    private ElapsedTime shootingWait = new ElapsedTime();
    private int headingOffset = 0;
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                follower.update();
                common.runAuton();
                common.runAprilTags();
                common.correctPose(follower);
                sendTelemetry();
            }
        }
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        common = new RobotCommon();
        common.initialize(hardwareMap);
        common.initAprilTag(hardwareMap);
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
        if (gamepad1.a) {
            common.setIntakeDirection(RobotCommon.ShaftDirection.IN);
        } else if (gamepad1.x) {
            common.setIntakeDirection(RobotCommon.ShaftDirection.OUT);
        } else if (gamepad1.b) {
            common.setShooterTarget(FULL_SHOOTING);
            common.setIntakeDirection(RobotCommon.ShaftDirection.IN);
            if (shootingWait.milliseconds() >= 750) {
                common.setFeederDirection(RobotCommon.ShaftDirection.IN);
            }
        } else if (gamepad1.left_bumper) {
            common.setFeederDirection(RobotCommon.ShaftDirection.OUT);
            common.setIntakeDirection(RobotCommon.ShaftDirection.OUT);
            common.setShooterTarget(FULL_BOWLING);
        } else {
            common.setFeederDirection(RobotCommon.ShaftDirection.STOP);
            common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
            common.setShooterTarget(0);
        }
        if (!gamepad1.b) {
            shootingWait.reset();
        }
        double speed = ROBOT_SLOW;
        double rotSpeed = ROT_SLOW;
        double x = square(-gamepad1.left_stick_y) * speed;
        double y = square(gamepad1.left_stick_x) * speed;
        if (gamepad1.guide)  {
            headingOffset = (int) Math.toDegrees(follower.getHeading());
            blackboard.put("headingOffset", headingOffset);
        }
        double rot = square(gamepad1.right_trigger-gamepad1.left_trigger) * rotSpeed;
        if (gamepad1.dpad_right) {
            rot = rotSpeed;
        } else if (gamepad1.dpad_left) {
            rot = -rotSpeed;
        }

        follower.setTeleOpDrive(x, -y, -rot, false, Math.toRadians(headingOffset));


    }
    public static double square(double amount) {
        return amount * Math.abs(amount);
    }
}