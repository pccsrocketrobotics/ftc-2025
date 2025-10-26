package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotCommon {

    public static int MAXWHEELSPEED = 5500;
    private static int INTAKE_SPEED = 1500;
    public static int LIFT_MAX;
    private double vx;
    private double vy;
    private double rot;
    public double intakePower;
    private double frontLeftTarget;
    private double backLeftTarget;
    private double frontRightTarget;
    private double backRightTarget;
    private int liftTargetPosition;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx intake;
    private DcMotorEx leftLift;
    private DcMotorEx rightLift;
    public GoBildaPinpointDriver odo;

    public void initialize(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        liftTargetPosition = rightLift.getCurrentPosition();
        runLift();
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void run() {
        runDrive();
        runIntake();
        runLift();
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

        double fastest = Math.max(Math.max(Math.abs(frontLeftTarget), Math.abs(frontRightTarget)), Math.max(Math.abs(backLeftTarget), Math.abs(backRightTarget)));
        if (fastest > MAXWHEELSPEED) {
            double scaleFactor = fastest / MAXWHEELSPEED;
            frontLeftTarget = frontLeftTarget / scaleFactor;
            frontRightTarget = frontRightTarget / scaleFactor;
            backLeftTarget = backLeftTarget / scaleFactor;
            backRightTarget = backRightTarget / scaleFactor;
        }

        frontLeft.setVelocity(frontLeftTarget);
        backLeft.setVelocity(backLeftTarget);
        frontRight.setVelocity(frontRightTarget);
        backRight.setVelocity(backRightTarget);
    }

    private void runIntake() {
        intake.setPower(intakePower);
    }

    public void setIntakePower(double incomingPower) {
        intakePower = incomingPower;

    }

    public void runLift() {
        rightLift.setTargetPosition(liftTargetPosition);
        leftLift.setTargetPosition(liftTargetPosition);
    }

    public void setLiftTargetPosition(int liftTargetPos) {
        liftTargetPosition = liftTargetPos;
    }

    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Heading", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("vx", vx);
        telemetry.addData("vy", vy);
        telemetry.addData("rot", rot);
        telemetry.addData("frontLeft", String.format("%d / %d", (int) frontLeft.getVelocity(), (int) frontLeftTarget));
        telemetry.addData("backLeft", String.format("%d / %d", (int) frontRight.getVelocity(), (int) frontRightTarget));
        telemetry.addData("frontRight", String.format("%d / %d", (int) backLeft.getVelocity(), (int) backLeftTarget));
        telemetry.addData("backRight", String.format("%d / %d", (int) backRight.getVelocity(), (int) backRightTarget));
        telemetry.addData("leftLift", leftLift.getCurrentPosition());
        telemetry.addData("rightLift", rightLift.getCurrentPosition());
        telemetry.addData("liftTargetPosition", liftTargetPosition);
        telemetry.update();
    }

}
