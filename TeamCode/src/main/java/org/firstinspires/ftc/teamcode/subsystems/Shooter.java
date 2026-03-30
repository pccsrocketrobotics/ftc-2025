package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import java.util.function.Supplier;

@Config
public class Shooter extends SubsystemBase {
    private final DcMotorEx shooter;
    public static PIDFCoefficients shooterCoefficients = new PIDFCoefficients(0.004, 0, 0, 0.0005);
    private final PIDFController shooterController = new PIDFController(shooterCoefficients);
    private double shooterTarget = 0;
    private double shooterPower = 0;
    private double cachedVelocity = 0;
    private boolean jamFix = false;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        cachedVelocity = shooter.getVelocity();
    }

    public double getVelocity() {
        return cachedVelocity;
    }

    public double getTarget() {
        return shooterTarget;
    }

    public double getPower() {
        return shooterPower;
    }

    public Command shootCommand(double velocity) {
        return runEnd(() -> {
            shooterTarget = velocity;
            shooterController.setTargetPosition(shooterTarget);
            shooterController.updatePosition(cachedVelocity);
            shooterController.updateFeedForwardInput(shooterTarget);
            shooterPower = shooterController.run();
            shooter.setPower(shooterPower);
        }, () -> {
            shooterTarget = 0;
            shooterPower = 0;
            shooter.setPower(0);
        });
    }

    public Command stopCommand() {
        // do nothing
        return run(() -> {});
    }

    public Command fixJamCommand() {
        return startEnd(
            () -> {
                jamFix = true;
                shooter.setPower(-1);
            },
            () -> {
                jamFix = false;
                shooter.setPower(0);
            }
        );
    }
}
