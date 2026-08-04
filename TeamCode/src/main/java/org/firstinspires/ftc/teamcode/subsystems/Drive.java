package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
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

    public Command go(
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier r
    ) {
        return run(
            () -> {
                double xval = x.getAsDouble();
                double yval = y.getAsDouble();
                double rval = r.getAsDouble();

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
