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

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "9 ball aligned with puzzle piece by small tri shoots from small tri")
@Config
public class BlueWallFar9 extends CommandOpMode {
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
    public static double SHOOTER_AUTON = 1540;
    public static long START_DELAY = 1000;

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
        PathChain ballAlignPath2 = follower.pathBuilder()
                .addPath(new BezierLine(farShotPose, alignPose2))
                .setLinearHeadingInterpolation(farShotPose.getHeading(), alignPose2.getHeading())
                .build();
        PathChain ballPickupPath2 = follower.pathBuilder()
                .addPath(new BezierLine(alignPose2, pickupPose2))
                .build();
        PathChain shootingPath3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, farShotPose))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), farShotPose.getHeading())
                .build();
        PathChain endPath = follower.pathBuilder()
                .addPath(new BezierLine(farShotPose, endPose))
                .setLinearHeadingInterpolation(farShotPose.getHeading(), endPose.getHeading())
                .build();

        schedule(new SequentialCommandGroup(
                new ParallelRaceGroup(
                    intake.inCommand(),
                    shooter.shootCommand(SHOOTER_AUTON),
                    new SequentialCommandGroup(
                        drive.followPathCommand(shootingPath),
                        new WaitCommand(START_DELAY),
                        feeder.shootSequenceCommand(3),
                        drive.followPathCommand(ballAlignPath),
                        drive.followPathCommand(ballPickupPath),
                        drive.followPathCommand(shootingPath2),
                        feeder.shootSequenceCommand(3),
                        drive.followPathCommand(ballAlignPath2),
                        drive.followPathCommand(ballPickupPath2),
                        drive.followPathCommand(shootingPath3),
                        feeder.shootSequenceCommand(3)
                    )
                ),
                // drive to end (with shooter and intake off)
                drive.followPathCommand(endPath)
        ));

        setBlackboard();
    }

    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }
}
