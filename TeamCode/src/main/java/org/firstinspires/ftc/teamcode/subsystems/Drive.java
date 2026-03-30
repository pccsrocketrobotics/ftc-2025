package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;

import org.firstinspires.ftc.teamcode.commands.TeleopMovementCommand;

import java.util.function.Supplier;

public class Drive extends SubsystemBase {
    private final Follower follower;

    public Drive(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public Command followPathCommand(PathChain path) {
        return new FollowPathCommand(follower, path);
    }

    public Command followPathCommand(PathChain path, boolean holdEnd, double maxPower) {
        return new FollowPathCommand(follower, path, holdEnd, maxPower);
    }

    public Command holdPointCommand(Pose pose) {
        return new HoldPointCommand(follower, pose, true);
    }

    public TeleopMovementCommand teleopDriveCommand(Supplier<Double> forwardSupplier,
                                                     Supplier<Double> lateralSupplier,
                                                     Supplier<Double> rotationSupplier,
                                                     Supplier<Double> headingOffsetSupplier) {
        TeleopMovementCommand cmd = new TeleopMovementCommand(follower,
                forwardSupplier, lateralSupplier, rotationSupplier, headingOffsetSupplier);
        cmd.addRequirements(this);
        return cmd;
    }

    public Command goToPoseCommand(Supplier<Pose> target) {
        return new InstantCommand(() -> {
            Pose t = target.get();
            PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), t))
                    .setConstantHeadingInterpolation(t.getHeading())
                    .setTimeoutConstraint(1500)
                    .build();
            follower.followPath(path);
        }, this);
    }

    public Command startTeleOpDriveCommand() {
        return new InstantCommand(() -> follower.startTeleOpDrive(true), this);
    }
}
