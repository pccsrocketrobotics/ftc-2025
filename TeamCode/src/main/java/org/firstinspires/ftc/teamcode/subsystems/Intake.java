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

    // A factory method that returns a Command.
    // put in initalize setDefaultCommand(inCommand());
    public Command inCommand() {
        return startEnd(
                () -> servo.setPower(0.7),
                () -> servo.setPower(0)
        );
    }
    public Command outCommand() {
        return startEnd(
                () -> servo.setPower(-0.7),
                () -> servo.setPower(0)
        );
    }
    public Command stopCommand() {
        return startEnd(
                () -> servo.setPower(0),    // runs once when the command starts
                null
        );
    }
}
