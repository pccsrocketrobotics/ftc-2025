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

    @Override
    public void periodic() {
        if (direction == ShaftDirection.IN) {
            intake.setPower(0.7);
            agitator.setPower(-1);
        } else if (direction == ShaftDirection.OUT) {
            intake.setPower(-1);
            agitator.setPower(0);
        } else {
            intake.setPower(0);
            agitator.setPower(0);
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
}
