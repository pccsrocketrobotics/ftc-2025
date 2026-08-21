package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.nio.file.Path;

import Ori.Coval.Logging.Logger.KoalaLog;

@Autonomous
public class CommandAuto extends CommandOpMode {

    private Follower follower;
    protected Pose startingPose = new Pose(-63,16.8,Math.toRadians(0));
    protected Pose pickupPose = new Pose(-12.7,50,Math.toRadians(90));


    @Override
    public void initialize() {

        // 1. Create subsystems
        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // 2. Register them so periodic() runs
        register(drive, intake);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        KoalaLog.setup(hardwareMap);

        PathChain testPath = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, pickupPose))
                .build();

        Command auton = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                            new WaitCommand(5000),
                            new FollowPathCommand(follower, testPath)
                    ),
                    intake.inCommand()

                ),
                // other stuff
                new WaitCommand(5000)
        );

        waitForStart();

        schedule(auton);





    }
}
