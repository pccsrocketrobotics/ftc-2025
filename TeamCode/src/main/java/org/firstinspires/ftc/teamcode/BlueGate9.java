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

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "9 ball corner against goal")
@Config
public class BlueGate9 extends CommandOpMode {
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    protected Drive drive;
    protected Shooter shooter;
    protected Intake intake;
    protected Feeder feeder;
    protected Follower follower;

    protected Pose startingPose = new Pose(48.1, 50.5, Math.toRadians(0));
    protected Pose halfShotPose = new Pose(27.5, 27, Math.toRadians(45));
    protected Pose alignPose1 = new Pose(11.7, 26.8, Math.toRadians(90));
    protected Pose pickupPose1 = new Pose(11.7, 50, Math.toRadians(90));
    protected Pose alignPose2 = new Pose(-13.6, 26.3, Math.toRadians(90));
    protected Pose pickupPose2 = new Pose(-12.7, 50, Math.toRadians(90));
    protected Pose halfShotPose2 = new Pose(34, 20.5, Math.toRadians(54));
    public static double SHOOTER_AUTON = 1325;

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
                .addPath(new BezierLine(startingPose, halfShotPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), halfShotPose.getHeading())
                .build();
        PathChain ballAlignPath = follower.pathBuilder()
                .addPath(new BezierLine(halfShotPose, alignPose1))
                .setLinearHeadingInterpolation(halfShotPose.getHeading(), alignPose1.getHeading())
                .build();
        PathChain ballPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(alignPose1, pickupPose1))
                .build();
        PathChain shootingPath2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose1, halfShotPose))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), halfShotPose.getHeading())
                .build();
        PathChain ballAlignPath2 = follower.pathBuilder()
                .addPath(new BezierLine(halfShotPose, alignPose2))
                .setLinearHeadingInterpolation(halfShotPose.getHeading(), alignPose2.getHeading())
                .build();
        PathChain ballPickupPath2 = follower.pathBuilder()
                .addPath(new BezierLine(alignPose2, pickupPose2))
                .build();
        PathChain shootingPath3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, halfShotPose2))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), halfShotPose2.getHeading())
                .build();

        schedule(new ParallelRaceGroup(
                intake.inCommand(),
                shooter.shootCommand(SHOOTER_AUTON),
                new SequentialCommandGroup(
                    drive.followPathCommand(shootingPath),
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
        ));

        setBlackboard();
    }

    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }
}

