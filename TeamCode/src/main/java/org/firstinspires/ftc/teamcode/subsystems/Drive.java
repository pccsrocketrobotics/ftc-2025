package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
    public static double SLOW_SPEED = 0.5;
    private double speedMult  = SLOW_SPEED;
    private final DcMotorEx lf;
    private final DcMotorEx rf;
    private final DcMotorEx lr;
    private final DcMotorEx rr;


    public Drive(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
    }
    public void setSpeedMult(double val) {
        speedMult = val;
    }

    public Command go(
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier r
    ) {
        return run(
            () -> {
                double xval = x.getAsDouble() * speedMult;
                double yval = y.getAsDouble() * speedMult;
                double rval = r.getAsDouble() * speedMult;

                double lfspeed = xval + yval + rval;
                double rfspeed = xval - yval - rval;
                double lrspeed = xval - yval + rval;
                double rrspeed = xval + yval - rval;

                lf.setPower(lfspeed);
                rf.setPower(rfspeed);
                lr.setPower(lrspeed);
                rr.setPower(rrspeed);
            }
        );
    }
}
