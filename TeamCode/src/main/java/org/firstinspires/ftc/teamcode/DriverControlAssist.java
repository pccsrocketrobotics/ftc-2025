package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.command.Robot;

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
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

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
        POSE_TRIGGER,
        POSE_PARK
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

        // Robot.setBulkReading(hardwareMap, MANUAL);

        initializeBindings();
    }

    private void initializeBindings() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        // Default commands
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

        intake.setDefaultCommand(intake.inCommand());

        // --- Operator bindings ---

        // Intake
        operator.getGamepadButton(GamepadKeys.Button.A).whenHeld(intake.stopCommand());
        operator.getGamepadButton(GamepadKeys.Button.B).whenHeld(intake.outCommand());

        // Shooter
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(shooter.shootCommand(SHOOTER_Y));
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(shooter.shootCommand(SHOOTER_X));
        operator.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(shooter.shootCommand(SHOOTER_BACK));
        operator.getGamepadButton(GamepadKeys.Button.PS).whenPressed(shooter.stopCommand());

        // Jam fix
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenHeld(shooter.fixJamCommand().alongWith(feeder.outCommand()));

        // Feeder
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(feeder.inCommand());
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(feeder.outCommand());

        // --- Driver bindings ---

        // Lift
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(lift.adjustCommand(LIFT_CHANGE));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(lift.adjustCommand(-LIFT_CHANGE));

        // Heading offset reset
        driver.getGamepadButton(GamepadKeys.Button.PS).whenPressed(new InstantCommand(() -> {
            headingOffset = (int) Math.toDegrees(follower.getHeading());
            blackboard.put("headingOffset", headingOffset);
        }));

        // Driver assist
        driver.getGamepadButton(GamepadKeys.Button.X).whenHeld(assistPoseCommand(AssistPose.POSE_X));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenHeld(assistPoseCommand(AssistPose.POSE_Y));
        driver.getGamepadButton(GamepadKeys.Button.BACK).whenHeld(assistPoseCommand(AssistPose.POSE_BACK));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(assistPoseCommand(AssistPose.POSE_TRIGGER));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(assistPoseCommand(AssistPose.POSE_TRIGGER));
        driver.getGamepadButton(GamepadKeys.Button.B).whenHeld(assistPoseCommand(AssistPose.POSE_PARK));

        // Pose heading adjustment (both gamepads)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> updatePose(1)));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> updatePose(-1)));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> updatePose(1)));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> updatePose(-1)));

    }

    @Override
    public void run() {
        super.run();
        sendTelemetry();
    }

    private Command assistPoseCommand(AssistPose assist) {
        return new StartEndCommand(
            () -> {
                lastAssist = assist;
                Pose target = getAssistPose(assist);
                if (headingOffset < 0) {
                    target = PoseUtil.mirror(target);
                }
                PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), target))
                    .setConstantHeadingInterpolation(target.getHeading())
                    .setTimeoutConstraint(1500)
                    .build();
                follower.followPath(path);
            },
            () -> {},
            drive
        );
    }

    private Pose getAssistPose(AssistPose assist) {
        switch (assist) {
            case POSE_X: return halfShotPose;
            case POSE_Y: return midShotPose;
            case POSE_BACK: return farShotPose;
            case POSE_TRIGGER: return closeShotPose;
            case POSE_PARK: return parkingPose;
        }
        return farShotPose;
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