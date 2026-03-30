package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

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

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "6 ball aligned with puzzle piece by small tri shoots from small tri")
@Config
public class BlueWallFar extends CommandOpMode {
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    protected Drive drive;
    protected Shooter shooter;
    protected Intake intake;
    protected Feeder feeder;
    protected Follower follower;

    protected Pose startingPose = new Pose(-60, 16.8, Math.toRadians(0));
    protected Pose farShotPose = new Pose(-53.5, 13.5, Math.toRadians(23.5));
    protected Pose alignPose3 = new Pose(-34, 26.3, Math.toRadians(90));
    protected Pose pickupPose3 = new Pose(-34, 50, Math.toRadians(90));
    public static double SHOOTER_AUTON = 1540;
    public static long START_DELAY = 3000;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        RobotDrawing.setDashboardTelemetry(FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        blackboard.put("follower", follower);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        feeder = new Feeder(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        ColorSensor colorSensor = new ColorSensor(hardwareMap);
        LEDs leds = new LEDs(hardwareMap, shooter, colorSensor);
        drive = new Drive(follower);

        register(drive, shooter, intake, feeder, lift, colorSensor, leds);

        PathChain shootingPath = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, farShotPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), farShotPose.getHeading())
                .build();
        PathChain ballAlignPath = follower.pathBuilder()
                .addPath(new BezierLine(farShotPose, alignPose3))
                .setLinearHeadingInterpolation(farShotPose.getHeading(), alignPose3.getHeading())
                .build();
        PathChain ballPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(alignPose3, pickupPose3))
                .build();
        PathChain shootingPath2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose3, farShotPose))
                .setLinearHeadingInterpolation(pickupPose3.getHeading(), farShotPose.getHeading())
                .build();

        schedule(new SequentialCommandGroup(
                // Start intake + shooter, drive to shooting pose, wait for START_DELAY
                new ParallelCommandGroup(
                        intake.inCommand(),
                        shooter.shootCommand(SHOOTER_AUTON),
                        drive.followPathCommand(shootingPath),
                        new WaitCommand(START_DELAY)
                ),
                // First round: 3 shots
                feeder.shootSequenceCommand(3),
                // Drive to ball align, then pickup
                drive.followPathCommand(ballAlignPath),
                drive.followPathCommand(ballPickupPath),
                // Drive back to shooting pose
                drive.followPathCommand(shootingPath2),
                // Second round: 3 shots
                feeder.shootSequenceCommand(3),
                // Stop and drive to end (align pose)
                new ParallelCommandGroup(
                        shooter.stopCommand(),
                        intake.stopCommand()
                ),
                drive.followPathCommand(ballAlignPath)
        ));

        setBlackboard();
    }

    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }
}
