package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class Feeder extends SubsystemBase {
    public enum ShaftDirection { STOP, IN, OUT }

    private final CRServo leftFeeder;
    private final CRServo rightFeeder;
    private ShaftDirection direction = ShaftDirection.STOP;
    private boolean fixJam = false;

    public Feeder(HardwareMap hardwareMap) {
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder.setDirection(CRServo.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        if (fixJam) {
            rightFeeder.setPower(-1);
            leftFeeder.setPower(-1);
            return;
        }
        if (direction == ShaftDirection.IN) {
            rightFeeder.setPower(1);
            leftFeeder.setPower(1);
        } else if (direction == ShaftDirection.OUT) {
            rightFeeder.setPower(-1);
            leftFeeder.setPower(-1);
        } else {
            rightFeeder.setPower(0);
            leftFeeder.setPower(0);
        }
    }

    public Command inCommand() {
                // NOTE: startend command that updates the motor power directly instead of using a supplier to avoid multiple motor writes in periodic

        return new InstantCommand(() -> direction = ShaftDirection.IN, this);
    }

    public Command outCommand() {
                // NOTE: startend command that updates the motor power directly instead of using a supplier to avoid multiple motor writes in periodic

        return new InstantCommand(() -> direction = ShaftDirection.OUT, this);
    }

        // NOTE: delete this as other commands should stop the intake on end

    public Command stopCommand() {
        return new InstantCommand(() -> direction = ShaftDirection.STOP, this);
    }

    public Command setFixJamCommand(boolean fix) {
        // NOTE: startend command that reverses the motor directly
        return new InstantCommand(() -> fixJam = fix, this);
    }

    public Command feedOneShotCommand() {
        return inCommand().withTimeout(700).andThen(new WaitCommand(400));
    }

    public Command shootSequenceCommand(int shots) {
        Command[] commands = new Command[shots];
        for (int i = 0; i < shots; i++) {
            commands[i] = feedOneShotCommand();
        }
        return new SequentialCommandGroup(commands);
    }
}
