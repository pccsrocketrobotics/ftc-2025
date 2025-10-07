package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestRobot extends LinearOpMode {
    public static int MAXWHEELSPEED = 5500;
    public static double ROBOT_SPEED = 1500;
    private double vx;
    private double vy;
    private double rot;
    private double frontLeftTarget;
    private double backLeftTarget;
    private double frontRightTarget;
    private double backRightTarget;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    @Override
    public void runOpMode() {
        initialize(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                sendTelemetry(telemetry);
            }
        }
    }

    public void initialize(HardwareMap hardwareMap) {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
    }
    private void controls() {
        double x = square(-gamepad1.left_stick_y) * ROBOT_SPEED;
        double y = square(gamepad1.left_stick_x) * ROBOT_SPEED;
        double rot = square(gamepad1.right_trigger-gamepad1.left_trigger) * ROBOT_SPEED;
        double vx = x;
        double vy = y;
        }

//    public void setRobotSpeed(double vx, double vy, double rot) {
//        this.vx = vx;
//        this.vy = vy;
//        this.rot = rot;
//    }

    private void runDrive() {
        frontLeftTarget = vx + vy + rot;
        backLeftTarget = (vx - vy) + rot;
        frontRightTarget = (vx - vy) - rot;
        backRightTarget = (vx + vy) - rot;

        frontLeft.setVelocity(frontLeftTarget);
        backLeft.setVelocity(backLeftTarget);
        frontRight.setVelocity(frontRightTarget);
        backRight.setVelocity(backRightTarget);
    }
    public static double square(double amount) {
        return amount * Math.abs(amount);

    }
    private void sendTelemetry(Telemetry telemetry) {
        telemetry.addData( "vx", vx);
        telemetry.addData("vy", vy);
        telemetry.addData("rot", rot);
        telemetry.addData("frontLeft", String.format("%d / %d", (int) frontLeft.getVelocity(), (int) frontLeftTarget));
        telemetry.addData("backLeft", String.format("%d / %d", (int) frontRight.getVelocity(), (int) frontRightTarget));
        telemetry.addData("frontRight", String.format("%d / %d", (int) backLeft.getVelocity(), (int) backLeftTarget));
        telemetry.addData("backRight", String.format("%d / %d", (int) backRight.getVelocity(), (int) backRightTarget));
        telemetry.update();
    }
}

