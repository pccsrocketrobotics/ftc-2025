package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
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

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "Pick up from loading zone and shoot from small tri")
@Config
public class BluePickle extends CommandOpMode {
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    protected Drive drive;
    protected Shooter shooter;
    protected Intake intake;
    protected Feeder feeder;
    protected Follower follower;

    protected Pose startingPose = new Pose(-60, 16.8, Math.toRadians(0));
    protected Pose farShotPose = new Pose(-53.5, 13.5, Math.toRadians(23.5));
    protected Pose loadingPose1 = new Pose(-58, 66, Math.toRadians(170));
    protected Pose loadingPose2 = new Pose(-58, 62, Math.toRadians(170));
    protected Pose loadingFace = new Pose(-72, 64, Math.toRadians(0));
    protected Pose loadingControl1Pose = new Pose(-48, 17);
    protected Pose loadingControl2Pose = new Pose(-40, 64);
    protected Pose slamPose1 = new Pose(-58, 61, Math.toRadians(105));
    protected Pose slamBackPose = new Pose(-60, 48, Math.toRadians(90));
    protected Pose slamPose2 = new Pose(-62, 62, Math.toRadians(90));
    protected Pose shotControlPose = new Pose(-52, 50);
    protected Pose endPose = new Pose(-60, 60, Math.toRadians(90));
    public static double SHOOTER_AUTON = 1540;
    public static long START_DELAY = 1500;
    public static long PICKUP_TIME1 = 2000;
    public static long PICKUP_TIME2 = 2000;

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
        PathChain shootingPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        loadingPose1,
                        shotControlPose,
                        farShotPose
                ))
                .setConstantHeadingInterpolation(farShotPose.getHeading())
                .build();
        PathChain loadingZonePath2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farShotPose,
                        loadingControl1Pose,
                        loadingControl2Pose,
                        loadingPose2
                ))
                .setHeadingInterpolation(PoseUtil.facingPoint(loadingFace.getX(), loadingFace.getY()))
                .build();
        PathChain shootingPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        loadingPose2,
                        shotControlPose,
                        farShotPose
                ))
                .setConstantHeadingInterpolation(farShotPose.getHeading())
                .build();
        PathChain endPath = follower.pathBuilder()
                .addPath(new BezierLine(farShotPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                    intake.inCommand(),
                    shooter.shootCommand(SHOOTER_AUTON),
                    new SequentialCommandGroup(
                        drive.followPathCommand(shootingPath),
                        new WaitCommand(START_DELAY),
                        feeder.shootSequenceCommand(3),
                        drive.followPathCommand(loadingZonePath),
                        new WaitCommand(PICKUP_TIME1),
                        drive.followPathCommand(slamBackPath),
                        new WaitCommand(PICKUP_TIME1),
                        drive.followPathCommand(shootingPath2),
                        feeder.shootSequenceCommand(3),
                        drive.followPathCommand(loadingZonePath2).withTimeout(PICKUP_TIME2),
                        drive.followPathCommand(shootingPath3),
                        feeder.shootSequenceCommand(3)
                    )
                ),
                drive.followPathCommand(endPath)
        ));

        setBlackboard();
    }

    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }
}
