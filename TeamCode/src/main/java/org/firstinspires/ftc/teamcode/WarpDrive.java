package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class WarpDrive extends LinearOpMode {
    private RobotCommon common;
    public static double ROBOT_SPEED = 1250;
    public static double WARP_SPEED = 5000;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                common.run();
                common.odo.update();
                common.sendTelemetry(telemetry);
            }
        }
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common = new RobotCommon();
        common.initialize(hardwareMap);
        common.odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
    }

    private void controls() {
        double speed = gamepad1.a ? WARP_SPEED : ROBOT_SPEED;
        double x = square(-gamepad1.left_stick_y) * speed;
        double y = square(gamepad1.left_stick_x) * speed;
        double heading = common.odo.getHeading(AngleUnit.RADIANS);
        double vx = x * Math.cos(heading) - y * Math.sin(heading);
        double vy = x * Math.sin(heading) + y * Math.cos(heading);
        if (gamepad1.guide)  {
            common.odo.resetPosAndIMU();
        }
        double rot = square(gamepad1.right_trigger-gamepad1.left_trigger) * ROBOT_SPEED;

        common.setRobotSpeed(vx, vy, rot);
    }

    public static double square(double amount) {
        return amount * Math.abs(amount);

    }
}

