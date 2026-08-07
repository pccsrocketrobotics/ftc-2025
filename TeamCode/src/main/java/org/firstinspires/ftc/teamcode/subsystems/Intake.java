package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CRServo servo;

    // Constructor: grab your hardware here.
    public Intake(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "intake");
    }
    public void spin(double speed) {
        servo.setPower(speed);
    }

    // A factory method that returns a Command.
    // put in initalize setDefaultCommand(inCommand());
    public Command inCommand() {
        return startEnd(
            () -> spin(0.7),
            () -> spin(0)
        );
    }
    public Command outCommand() {
        return startEnd(
            () -> spin(-0.7),
            () -> spin(0)
        );
    }
    public Command stopCommand() {
        return startEnd(
            () -> spin(0),
            () -> {}
        );
    }
}
