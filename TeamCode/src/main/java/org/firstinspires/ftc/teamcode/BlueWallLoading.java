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

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "Gets loading zone and last set of balls")
@Config
public class BlueWallLoading extends CommandOpMode {
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
    protected Pose alignPose2 = new Pose(-10, 26.3, Math.toRadians(90));
    protected Pose pickupPose2 = new Pose(-10, 50, Math.toRadians(90));
    protected Pose endPose = new Pose(-34, 26.3, Math.toRadians(0));
    protected Pose slamPose1 = new Pose(-58, 61, Math.toRadians(105));
    protected Pose slamBackPose = new Pose(-60, 48, Math.toRadians(90));
    protected Pose slamPose2 = new Pose(-62, 62, Math.toRadians(90));

    public static double SHOOTER_AUTON = 1540;
    public static long START_DELAY = 1000;
    public static long PICKUP_TIME1 = 2000;

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
        PathChain loadingZonePath = follower.pathBuilder()
                .addPath(new BezierLine(farShotPose, slamPose1))
                .setConstantHeadingInterpolation(slamPose1.getHeading())
                .build();
        PathChain slamBackPath = follower.pathBuilder()
                .addPath(new BezierLine(slamPose1, slamBackPose))
                .setConstantHeadingInterpolation(slamBackPose.getHeading())
                .addPath(new BezierLine(slamBackPose, slamPose2))
                .setConstantHeadingInterpolation(slamPose2.getHeading())
                .build();
        PathChain shootingPath3 = follower.pathBuilder()
                .addPath(new BezierLine(slamPose2, farShotPose))
                .setLinearHeadingInterpolation(slamPose2.getHeading(), farShotPose.getHeading())
                .build();
        PathChain endPath = follower.pathBuilder()
                .addPath(new BezierLine(farShotPose, endPose))
                .setLinearHeadingInterpolation(farShotPose.getHeading(), endPose.getHeading())
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
                // Loading zone: drive to slam pose, wait for pickup
                drive.followPathCommand(loadingZonePath),
                new WaitCommand(PICKUP_TIME1),
                // Slam back and forward
                drive.followPathCommand(slamBackPath),
                new WaitCommand(PICKUP_TIME1),
                // Drive back to shooting pose
                drive.followPathCommand(shootingPath3),
                // Third round: 3 shots
                feeder.shootSequenceCommand(3),
                // Stop and drive to end
                new ParallelCommandGroup(
                        shooter.stopCommand(),
                        intake.stopCommand()
                ),
                drive.followPathCommand(endPath)
        ));

        setBlackboard();
    }

    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }
}
