package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotCommon {

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

    public void initialize(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run() {
        runDrive();
    }

    public void setRobotSpeed(double vx, double vy, double rot) {
        this.vx = vx;
        this.vy = vy;
        this.rot = rot;
    }

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

    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData( "vx", vx);
        telemetry.addData("vy", vy);
        telemetry.addData("rot", rot);
        telemetry.addData("frontLeftTarget", frontLeftTarget);
        telemetry.addData("backLeftTarget", backLeftTarget);
        telemetry.addData("frontRightTarget", frontRightTarget);
        telemetry.addData("backRightTarget", backRightTarget);
        telemetry.update();
    }

}

