package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

@Config
public class Shooter extends SubsystemBase {
    private final DcMotorEx shooter;
    public static PIDFCoefficients shooterCoefficients = new PIDFCoefficients(0.004, 0, 0, 0.0005);
    private final PIDFController shooterController = new PIDFController(shooterCoefficients);
    private double shooterTarget = 0;
    private boolean fixJam = false;
    private double shooterPower = 0;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        if (fixJam) {
            shooter.setPower(-1);
            return;
        }
        shooterController.setTargetPosition(shooterTarget);
        shooterController.updatePosition(shooter.getVelocity());
        shooterController.updateFeedForwardInput(shooterTarget);
        shooterPower = shooterController.run();
        if (shooterTarget == 0) {
            shooter.setPower(0);
        } else {
            shooter.setPower(shooterPower);
        }
    }

    public double getVelocity() {
        // NOTE: cache getVelocity in a variable in periodic to avoid multiple hardware reads
        return shooter.getVelocity();
    }

    public double getTarget() {
        return shooterTarget;
    }

    public double getPower() {
        return shooterPower;
    }

    public Command shootCommand(double velocity) {
        // NOTE: run command that updates the controller and sets the motor power
        // NOTE: it should take a supplier
        return new InstantCommand(() -> shooterTarget = velocity, this);
    }

    public Command stopCommand() {
        // NOTE: startend command that sets the target to 0 and sets the power to 0 to avoid multiple motor writes in periodic
        return new InstantCommand(() -> shooterTarget = 0, this);
    }

    public Command setFixJamCommand(boolean fix) {
        // NOTE: startend command that reverses the motor directly
        return new InstantCommand(() -> fixJam = fix, this);
    }

    public Command waitUntilAtTargetCommand() {
        return new WaitUntilCommand(() -> Math.abs(shooter.getVelocity() - shooterTarget) < 100);
    }
}
