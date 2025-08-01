package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DriverControl extends LinearOpMode {
    private RobotCommon common;
    GoBildaPinpointDriver odo;
    public static double ROBOT_SPEED = 2000;

    @Override
    public void runOpMode() {
        initialize();
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setOffsets(102, -98.6);
        odo.resetPosAndIMU();

        waitForStart();
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                controls();

                odo.update();
                common.run();

                telemetry.addData("deviceStatus", odo.getDeviceStatus());
                telemetry.addData("loopTime", odo.getLoopTime());
                telemetry.addData("encoderX", odo.getEncoderX());
                telemetry.addData("encoderY", odo.getEncoderY());
                telemetry.addData("posX", odo.getPosX());
                telemetry.addData("posY", odo.getPosY());
                telemetry.addData("heading", AngleUnit.DEGREES.fromRadians(odo.getHeading()));
                telemetry.addData("velX", odo.getVelX());
                telemetry.addData("velY", odo.getVelY());
                telemetry.addData("headingVel", AngleUnit.DEGREES.fromRadians(odo.getHeadingVelocity()));
                common.sendTelemetry(telemetry);
            }
        }
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common = new RobotCommon();
        common.initialize(hardwareMap);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }

    private void controls() {
        double vx = square(-gamepad1.left_stick_y) * ROBOT_SPEED;
        double vy = square(gamepad1.right_stick_x) * ROBOT_SPEED;
        double rot = square(gamepad1.right_trigger-gamepad1.left_trigger) * ROBOT_SPEED;

        common.setRobotSpeed(vx, vy, rot);

    }

    public static double square(double amount) {
        return amount * Math.abs(amount);

    }
}
