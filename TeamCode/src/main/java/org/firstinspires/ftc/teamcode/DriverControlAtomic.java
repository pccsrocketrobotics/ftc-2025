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
public class DriverControlAtomic extends LinearOpMode {
    private RobotCommon common;
    private Follower follower;
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();

    private enum AssistPose {
        POSE_X,
        POSE_Y,
        POSE_BACK,
        POSE_TRIGGER
    }

    private AssistPose lastAssist = AssistPose.POSE_BACK;
    public static double ROBOT_SLOW = 0.5;
    public static double ROBOT_FAST = 1;
    public static double ROT_FAST = 0.5;
    public static double ROT_SLOW = 0.3;
    public static int LIFT_MAX = 2800;
    public static double SHOOTER_X = 1375;
    public static double SHOOTER_Y = 1400;
    public static double SHOOTER_BACK = 1480;
    public static int LIFT_CHANGE = 50;
    private int headingOffset = 0;
    private boolean isBlueAlliance = true;
    protected Pose xPose = new Pose(27.5, 27, Math.toRadians(45));
    //fixed
    protected Pose rightBumperPose = new Pose(-44, -35, Math.toRadians(-162));
    protected Pose leftBumperPose = new Pose(-44, 35, Math.toRadians(18));
    protected Pose yPose = new Pose(0, 0, Math.toRadians(45));
    protected Pose aPose = new Pose(-120, -26, Math.toRadians(-135));
    protected Pose bPose = new Pose(-93, 0, Math.toRadians(-135));

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if (opModeIsActive()) {
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
            //noinspection DataFlowIssue
            headingOffset = (int) blackboard.get("headingOffset");
        }
        if (blackboard.containsKey("isBlueAlliance")) {
            //noinspection DataFlowIssue
            isBlueAlliance = (boolean) blackboard.get("isBlueAlliance");
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
            common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
        } else if (gamepad2.b) {
            common.setIntakeDirection(RobotCommon.ShaftDirection.OUT);
        } else {
            common.setIntakeDirection(RobotCommon.ShaftDirection.IN);
        }
        if (gamepad1.dpad_up) {
            int pos = common.getLiftTargetPosition() + LIFT_CHANGE;
            if (pos > LIFT_MAX) {
                pos = LIFT_MAX;
            }
            common.setLiftTargetPosition(pos);
        }
        if (gamepad1.dpad_down) {
            int pos = common.getLiftTargetPosition() - LIFT_CHANGE;
            if (pos < 0) {
                pos = 0;
            }
            common.setLiftTargetPosition(pos);
        }
        if (gamepad2.y) {
            common.setShooterTarget(SHOOTER_Y);
        } else if (gamepad2.x) {
            common.setShooterTarget(SHOOTER_X);
        } else if (gamepad2.back) {
            common.setShooterTarget(SHOOTER_BACK);
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

        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            updatePose(1);
        } else if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
            updatePose(-1);
        }
        if (gamepad1.x) {
            lastAssist = AssistPose.POSE_X;
            if (follower.isTeleopDrive()) {
                goToPose(xPose);
            }
        } else if (gamepad1.y) {
            lastAssist = AssistPose.POSE_Y;
            if (follower.isTeleopDrive()) {
                goToPose(yPose);
            }
        } else if (gamepad1.a) {
            if (follower.isTeleopDrive()) {
                goToPose(aPose);
            }
        } else if (gamepad1.b) {
            if (follower.isTeleopDrive()) {
                goToPose(bPose);
            }
        } else if (gamepad1.right_bumper) {
            lastAssist = AssistPose.POSE_BACK;
            if (follower.isTeleopDrive()) {
                goToPose(isBlueAlliance ? rightBumperPose : leftBumperPose);
            }
        } else if (gamepad1.left_bumper) {
            if (follower.isTeleopDrive()) {
                goToPose(isBlueAlliance ? leftBumperPose : rightBumperPose);
            }
        }
        else {
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
        if (gamepad1.guide) {
            headingOffset = (int) Math.toDegrees(follower.getHeading());
            blackboard.put("headingOffset", headingOffset);
        }
        double rot = square(gamepad1.right_trigger - gamepad1.left_trigger) * rotSpeed;

        follower.setTeleOpDrive(x, -y, -rot, false, Math.toRadians(headingOffset));

    }

    private void updatePose(int delta) {
        switch (lastAssist) {
            case POSE_X:
                xPose = xPose.setHeading(xPose.getHeading() + Math.toRadians(delta));
                break;
            case POSE_Y:
                yPose = yPose.setHeading(yPose.getHeading() + Math.toRadians(delta));
                break;
            case POSE_BACK:
                rightBumperPose = rightBumperPose.setHeading(rightBumperPose.getHeading() + Math.toRadians(delta));
                break;
            case POSE_TRIGGER:
                aPose = aPose.setHeading(aPose.getHeading() + Math.toRadians(delta));
                break;
        }
    }

    private void goToPose(Pose target) {
        if (!isBlueAlliance) {
            target = RobotCommon.mirror(target);
        }
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setConstantHeadingInterpolation(target.getHeading())
                .setTimeoutConstraint(1500)
                .build();
        follower.followPath(path);
    }

    public static double square(double amount) {
        return amount * Math.abs(amount);
    }
}