package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
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
    public double shooterTarget;
    private double rightFeederVelocity;
    private double leftFeederVelocity;
    public static double SHOOTER_P = 0;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0;
    public static double SHOOTER_F = 0.00063;
    private PIDFCoefficients shooterCoefficients = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
    private PIDFController shooterController = new PIDFController(shooterCoefficients);
    private double shooterPower;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    public enum FeederOptions {
        STOP, IN, OUT
    }
    private FeederOptions feederOption = FeederOptions.STOP;
    private DcMotorEx shooter;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx intake;
    private DcMotorEx leftLift;
    private DcMotorEx rightLift;
    private LED redLed;
    private LED blueLed;
    private LED yellowLed1;
    private LED yellowLed2;
    private ElapsedTime ledTimer = new ElapsedTime();
    public GoBildaPinpointDriver odo;

    public void initialize(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        redLed = hardwareMap.get(LED.class, "redLed");
        blueLed = hardwareMap.get(LED.class, "blueLed");
        yellowLed1 = hardwareMap.get(LED.class, "yellowLed1");
        yellowLed2 = hardwareMap.get(LED.class, "yellowLed2");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setDirection(CRServo.Direction.REVERSE);

        liftTargetPosition = rightLift.getCurrentPosition();
        runLift();
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        redLed.off();
        yellowLed1.off();
        yellowLed2.off();
        blueLed.off();
    }

    public void run() {
        runDrive();
        runIntake();
        runLift();
        runShooter();
        runFeeder();
        runLeds();
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
//Runners
    private void runIntake() {
        intake.setPower(intakePower);
    }

    private void runShooter() {
        shooterCoefficients.setCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooterController.setTargetPosition(shooterTarget);
        shooterController.updatePosition(shooter.getVelocity());
        shooterController.updateFeedForwardInput(shooterTarget);
        shooterPower = shooterController.run();
        if (shooterTarget == 0) {
            shooter.setPower(0);
        }else {
            shooter.setPower(shooterPower);
        }
    }

    private void runLift() {
        rightLift.setTargetPosition(liftTargetPosition);
        leftLift.setTargetPosition(liftTargetPosition);
    }
    private void runFeeder() {
        if(feederOption == FeederOptions.IN) {
            rightFeeder.setPower(1);
            leftFeeder.setPower(1);
        } else if(feederOption == FeederOptions.OUT) {
            rightFeeder.setPower(-1);
            leftFeeder.setPower(-1);
        } else {
            rightFeeder.setPower(0);
            leftFeeder.setPower(0);
        }
    }
    private void runLeds() {
        boolean red = false;
        boolean blue = false;
        boolean yellow = false;
        if (shooter.getVelocity() < 100 && shooterTarget == 0) {
            ledTimer.reset();
            red = true;
        } else if (Math.abs(shooter.getVelocity() - shooterTarget) < 50) {
            if (ledTimer.milliseconds() > 200) {
                blue = true;
            } else {
                yellow = true;
            }
        } else {
            ledTimer.reset();
            yellow = true;
        }
        redLed.enable(red);
        blueLed.enable(blue);
        yellowLed1.enable(yellow);
        yellowLed2.enable(yellow);
    }
    //Setters
    public void setLiftTargetPosition(int liftTargetPosition) {
        this.liftTargetPosition = liftTargetPosition;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }
    public void setShooterTarget(double shooterTarget) {
        this.shooterTarget = shooterTarget;
    }
    public void setFeederVelocity(FeederOptions feederOption) {
        this.feederOption = feederOption;
    }
    public void sendTelemetry(Telemetry telemetry) {
        //telemetry.addData("Heading", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("vx", vx);
        telemetry.addData("vy", vy);
        telemetry.addData("rot", rot);
        telemetry.addData("shooterTarget", shooterTarget);
        telemetry.addData("shooter", shooter.getVelocity());
        telemetry.addData("shooterPower", shooterPower);
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
