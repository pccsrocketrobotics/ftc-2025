package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.dashboard.DashboardTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.RobotDrawing;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
@Config
public class DriverControlAssist extends CommandOpMode {
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Feeder feeder;
    private Lift lift;
    private ColorSensor colorSensor;
    private Follower follower;

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
    public static double SHOOTER_X = 1375;
    public static double SHOOTER_Y = 1400;
    public static double SHOOTER_BACK = 1540;
    public static int LIFT_CHANGE = 50;
    private int headingOffset = 0;
    protected Pose halfShotPose = new Pose(27.5, 27, Math.toRadians(45));
    protected Pose farShotPose = new Pose(-54.5, 12.5, Math.toRadians(23.5));
    protected Pose midShotPose = new Pose(0, 0, Math.toRadians(45));
    protected Pose closeShotPose = new Pose(42, -4, Math.toRadians(75));
    protected Pose parkingPose = new Pose(-35.5, -35, Math.toRadians(180));

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        // Create follower from blackboard or fresh
        follower = (Follower) blackboard.get("follower");
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
        }
        follower.update();
        follower.startTeleOpDrive(true);

        if (blackboard.containsKey("headingOffset")) {
            headingOffset = (int) blackboard.get("headingOffset");
        }

        // Create subsystems
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        feeder = new Feeder(hardwareMap);
        lift = new Lift(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
        LEDs leds = new LEDs(hardwareMap, shooter, colorSensor);
        drive = new Drive(follower);

        register(drive, shooter, intake, feeder, lift, colorSensor, leds);

        // Default command: teleop drive
        drive.setDefaultCommand(drive.teleopDriveCommand(
            () -> {
                double speed = gamepad1.a ? ROBOT_SLOW : ROBOT_FAST;
                return square(-gamepad1.left_stick_y) * speed;
            },
            () -> {
                double speed = gamepad1.a ? ROBOT_SLOW : ROBOT_FAST;
                return -square(gamepad1.left_stick_x) * speed;
            },
            () -> {
                double rotSpeed = gamepad1.a ? ROT_SLOW : ROT_FAST;
                return -square(gamepad1.right_trigger - gamepad1.left_trigger) * rotSpeed;
            },
            () -> Math.toRadians(headingOffset)
        ));
    }

    @Override
    public void run() {
        controls();
        super.run();
        sendTelemetry();
    }

    private void controls() {
        // Intake
        if (gamepad2.a) {
            intake.inCommand().schedule();
        } else if (gamepad2.b) {
            intake.outCommand().schedule();
        } else {
            intake.stopCommand().schedule();
        }

        // Lift
        if (gamepad1.dpad_up) {
            lift.adjustCommand(LIFT_CHANGE).schedule();
        }
        if (gamepad1.dpad_down) {
            lift.adjustCommand(-LIFT_CHANGE).schedule();
        }

        // Shooter velocity presets
        if (gamepad2.y) {
            shooter.shootCommand(SHOOTER_Y).schedule();
        } else if (gamepad2.x) {
            shooter.shootCommand(SHOOTER_X).schedule();
        } else if (gamepad2.back) {
            shooter.shootCommand(SHOOTER_BACK).schedule();
        } else if (gamepad2.guide) {
            shooter.stopCommand().schedule();
        }

        // Jam fix
        if (gamepad2.dpad_down) {
            shooter.setFixJamCommand(true).schedule();
            feeder.setFixJamCommand(true).schedule();
        } else {
            shooter.setFixJamCommand(false).schedule();
            feeder.setFixJamCommand(false).schedule();
        }

        // Feeder
        if (gamepad2.right_bumper) {
            feeder.inCommand().schedule();
        } else if (gamepad2.left_bumper) {
            feeder.outCommand().schedule();
        } else {
            feeder.stopCommand().schedule();
        }

        // Pose heading adjustment
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            updatePose(1);
        } else if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
            updatePose(-1);
        }

        // Assist poses
        if (gamepad1.x) {
            lastAssist = AssistPose.POSE_X;
            if (follower.isTeleopDrive()) {
                goToPose(halfShotPose);
            }
        } else if (gamepad1.y) {
            lastAssist = AssistPose.POSE_Y;
            if (follower.isTeleopDrive()) {
                goToPose(midShotPose);
            }
        } else if (gamepad1.back) {
            lastAssist = AssistPose.POSE_BACK;
            if (follower.isTeleopDrive()) {
                goToPose(farShotPose);
            }
        } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
            lastAssist = AssistPose.POSE_TRIGGER;
            if (follower.isTeleopDrive()) {
                goToPose(closeShotPose);
            }
        } else if (gamepad1.b) {
            if (follower.isTeleopDrive()) {
                goToPose(parkingPose);
            }
        } else {
            if (!follower.isTeleopDrive()) {
                follower.startTeleOpDrive(true);
            }
        }

        // Heading offset reset
        if (gamepad1.guide) {
            headingOffset = (int) Math.toDegrees(follower.getHeading());
            blackboard.put("headingOffset", headingOffset);
        }
    }

    private void updatePose(int delta) {
        switch (lastAssist) {
            case POSE_X:
                halfShotPose = halfShotPose.setHeading(halfShotPose.getHeading() + Math.toRadians(delta));
                break;
            case POSE_Y:
                midShotPose = midShotPose.setHeading(midShotPose.getHeading() + Math.toRadians(delta));
                break;
            case POSE_BACK:
                farShotPose = farShotPose.setHeading(farShotPose.getHeading() + Math.toRadians(delta));
                break;
            case POSE_TRIGGER:
                closeShotPose = closeShotPose.setHeading(closeShotPose.getHeading() + Math.toRadians(delta));
                break;
        }
    }

    private void goToPose(Pose target) {
        if (headingOffset < 0) {
            target = PoseUtil.mirror(target);
        }
        drive.goToPoseCommand(() -> target).schedule();
    }

    private void sendTelemetry() {
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        RobotDrawing.draw(dashboardTelemetry.getCurrentPacket(), follower);
        telemetry.addData("headingOffset", headingOffset);
        telemetry.addData("isTeleopDrive", follower.isTeleopDrive());
        telemetry.addData("shooterTarget", shooter.getTarget());
        telemetry.addData("shooter", shooter.getVelocity());
        telemetry.addData("ball distance", colorSensor.getBallDistance());
        telemetry.update();
    }

    public static double square(double amount) {
        return amount * Math.abs(amount);
    }
}