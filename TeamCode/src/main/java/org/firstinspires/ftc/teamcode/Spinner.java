package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class Spinner extends LinearOpMode {
    private DcMotorEx flywheel;

    public static double SPIN = 2000;
    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = hardwareMap.get(DcMotorEx.class, "arm");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        boolean go = false;
        while(!isStopRequested()) {
            if (gamepad1.aWasPressed()) {
                go = !go;
            }

            flywheel.setVelocity(go ? SPIN : 0);
        }

    }
}
