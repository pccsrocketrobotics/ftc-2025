package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public enum ShaftDirection { STOP, IN, OUT }

    private final DcMotorEx intake;
    private final CRServo agitator;
    private ShaftDirection direction = ShaftDirection.STOP;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agitator = hardwareMap.get(CRServo.class, "agitator");
    }

    public Command inCommand() {
        return startEnd(() -> {
            direction = ShaftDirection.IN;
            intake.setPower(0.7);
            agitator.setPower(-1);
        }, () -> {
            direction = ShaftDirection.STOP;
            intake.setPower(0);
            agitator.setPower(0);
        });
    }

    public Command outCommand() {
        return startEnd(() -> {
            direction = ShaftDirection.OUT;
            intake.setPower(-1);
            agitator.setPower(0);
        }, () -> {
            direction = ShaftDirection.STOP;
            intake.setPower(0);
            agitator.setPower(0);
        });
    }

    public Command stopCommand() {
        // do nothing
        return run(() -> {});
    }
}
