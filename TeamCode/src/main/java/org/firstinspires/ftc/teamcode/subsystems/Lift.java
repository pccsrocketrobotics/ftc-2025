package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Lift extends SubsystemBase {
    public static int LIFT_VELOCITY = 5000;
    public static int LIFT_MAX = 2800;

    private final DcMotorEx leftLift;
    private final DcMotorEx rightLift;
    private int targetPosition;

    public Lift(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        targetPosition = rightLift.getCurrentPosition();
        rightLift.setTargetPosition(targetPosition);
        leftLift.setTargetPosition(targetPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void periodic() {
        // NOTE: nothing should be in this loop
        rightLift.setTargetPosition(targetPosition);
        leftLift.setTargetPosition(targetPosition);
        leftLift.setVelocity(LIFT_VELOCITY);
        rightLift.setVelocity(LIFT_VELOCITY);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public Command setPositionCommand(int position) {
        // NOTE: refactor to set the position of the motor directly using a supplier
        return new InstantCommand(() -> targetPosition = position, this);
    }

    public Command adjustCommand(int delta) {
        // NOTE: refactor to set the position of the motor directly using a supplier
        return new InstantCommand(() -> {
            targetPosition = Math.max(0, Math.min(LIFT_MAX, targetPosition + delta));
        }, this);
    }
}
