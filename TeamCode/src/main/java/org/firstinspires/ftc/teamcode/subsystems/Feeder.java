package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class Feeder extends SubsystemBase {
    public enum ShaftDirection { STOP, IN, OUT }

    private final CRServo leftFeeder;
    private final CRServo rightFeeder;
    private ShaftDirection direction = ShaftDirection.STOP;

    public Feeder(HardwareMap hardwareMap) {
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder.setDirection(CRServo.Direction.REVERSE);
    }

    public Command inCommand() {
        return startEnd(() -> {
            direction = ShaftDirection.IN;
            rightFeeder.setPower(1);
            leftFeeder.setPower(1);
        }, () -> {
            direction = ShaftDirection.STOP;
            rightFeeder.setPower(0);
            leftFeeder.setPower(0);
        });
    }

    public Command outCommand() {
        return startEnd(() -> {
            direction = ShaftDirection.OUT;
            rightFeeder.setPower(-1);
            leftFeeder.setPower(-1);
        }, () -> {
            direction = ShaftDirection.STOP;
            rightFeeder.setPower(0);
            leftFeeder.setPower(0);
        });
    }

    public Command shootSequenceCommand(int shots) {
        Command[] commands = new Command[shots];
        for (int i = 0; i < shots; i++) {
            commands[i] = inCommand().withTimeout(700);
            if (i < shots - 1) {
                commands[i] = commands[i].andThen(new WaitCommand(400));
            }
        }
        return new SequentialCommandGroup(commands);
    }
}
