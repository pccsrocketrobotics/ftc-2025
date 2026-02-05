package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    public static double FULL_SHOOTING = 900;
    public static double FULL_BOWLING = -900;
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
        } else if (gamepad1.b) {
            common.setIntakeDirection(RobotCommon.ShaftDirection.OUT);
        } else if (gamepad1.right_bumper) {
            common.setFeederDirection(RobotCommon.ShaftDirection.IN);
            common.setIntakeDirection(RobotCommon.ShaftDirection.IN);
            common.setShooterTarget(FULL_SHOOTING);
        } else if (gamepad1.left_bumper) {
            common.setFeederDirection(RobotCommon.ShaftDirection.OUT);
            common.setIntakeDirection(RobotCommon.ShaftDirection.OUT);
            common.setShooterTarget(FULL_BOWLING);
        } else {
            common.setFeederDirection(RobotCommon.ShaftDirection.STOP);
            common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
            common.setShooterTarget(0);
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

        follower.setTeleOpDrive(x, -y, -rot, false, Math.toRadians(headingOffset));


    }
    public static double square(double amount) {
        return amount * Math.abs(amount);
    }
}