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

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "6 ball aligned with puzzle piece by small tri shoots from big tri")
@Config
public class BlueWallMid extends CommandOpMode {
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    protected Drive drive;
    protected Shooter shooter;
    protected Intake intake;
    protected Feeder feeder;
    protected Follower follower;

    protected Pose startingPose = new Pose(-63, 16.8, Math.toRadians(0));
    protected Pose midShotPose = new Pose(7.7, 15, Math.toRadians(41.6));
    protected Pose alignPose2 = new Pose(-13.6, 26.3, Math.toRadians(90));
    protected Pose pickupPose2 = new Pose(-12.7, 50, Math.toRadians(90));
    public static double SHOOTER_AUTON = 1350;

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
                .addPath(new BezierLine(startingPose, midShotPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), midShotPose.getHeading())
                .build();
        PathChain ballAlignPath = follower.pathBuilder()
                .addPath(new BezierLine(midShotPose, alignPose2))
                .setLinearHeadingInterpolation(midShotPose.getHeading(), alignPose2.getHeading())
                .build();
        PathChain ballPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(alignPose2, pickupPose2))
                .build();
        PathChain shootingPath2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, midShotPose))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), midShotPose.getHeading())
                .build();

        schedule(new SequentialCommandGroup(
                // Start intake + shooter, drive to shooting pose
                new ParallelCommandGroup(
                        intake.inCommand(),
                        shooter.shootCommand(SHOOTER_AUTON),
                        drive.followPathCommand(shootingPath)
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
