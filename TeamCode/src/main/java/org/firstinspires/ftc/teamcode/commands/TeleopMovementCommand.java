package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.function.Supplier;

public class TeleopMovementCommand extends CommandBase {
    private final Follower follower;
    private final Supplier<Double> forwardSupplier;
    private final Supplier<Double> lateralSupplier;
    private final Supplier<Double> rotationSupplier;
    private final Supplier<Double> headingOffsetSupplier;

    public TeleopMovementCommand(Follower follower,
                                  Supplier<Double> forwardSupplier,
                                  Supplier<Double> lateralSupplier,
                                  Supplier<Double> rotationSupplier,
                                  Supplier<Double> headingOffsetSupplier) {
        this.follower = follower;
        this.forwardSupplier = forwardSupplier;
        this.lateralSupplier = lateralSupplier;
        this.rotationSupplier = rotationSupplier;
        this.headingOffsetSupplier = headingOffsetSupplier;
    }

    @Override
    public void initialize() {
        follower.startTeleOpDrive(true);
    }

    @Override
    public void execute() {
        follower.setTeleOpDrive(
                forwardSupplier.get(),
                lateralSupplier.get(),
                rotationSupplier.get(),
                false,
                headingOffsetSupplier.get()
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }
}
