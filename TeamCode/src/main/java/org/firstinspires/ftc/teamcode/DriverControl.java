package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriverControl extends LinearOpMode {
    private RobotCommon common;
    public static double ROBOT_SPEED = 2000;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                common.run();
                odo.update();
                common.sendTelemetry(telemetry);
            }
        }
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common = new RobotCommon();
        common.initialize(hardwareMap);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
    }

    private void controls() {
        double x = square(-gamepad1.left_stick_y) * ROBOT_SPEED;
        double y = square(gamepad1.left_stick_x) * ROBOT_SPEED;
        double heading = odo.getHeading();
        double vx = x * Math.cos(heading) - y * Math.sin(heading);
        double vy = x * Math.sin(heading) + y * Math.cos(heading);
        if (gamepad1.guide)  {
            odo.recalibrateIMU();
        }
        double rot = square(gamepad1.right_trigger-gamepad1.left_trigger) * ROBOT_SPEED;

        common.setRobotSpeed(vx, vy, rot);
        telemetry.addData("Heading", Math.toDegrees(heading));
    }

    public static double square(double amount) {
        return amount * Math.abs(amount);

    }
}
