package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriverControl extends LinearOpMode {
    private RobotCommon common;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                common.run();
                common.sendTelemetry(telemetry);
            }
        }

    }

    private void initialize() {
        common = new RobotCommon();
        common.initialize(hardwareMap);
    }

    private void controls() {
        double vx = square(-gamepad1.left_stick_y);
        double vy = square(gamepad1.right_stick_x);
        double rot = square(gamepad1.right_trigger-gamepad1.left_trigger);

        common.setRobotSpeed(vx, vy, rot);

    }

    public static double square(double amount) {
        return amount * Math.abs(amount);

    }
}
