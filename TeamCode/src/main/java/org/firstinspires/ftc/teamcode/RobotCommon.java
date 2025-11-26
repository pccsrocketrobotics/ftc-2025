package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class RobotCommon {

    public static int MAX_WHEEL_SPEED = 5500;
    public static int LIFT_VELOCITY = 5000;

    private double vx;
    private double vy;
    private double rot;
    public enum ShaftDirection {
        STOP, IN, OUT
    }
    public ShaftDirection intakeDirection;
    private double frontLeftTarget;
    private double backLeftTarget;
    private double frontRightTarget;
    private double backRightTarget;
    private int liftTargetPosition;
    public double shooterTarget;
    public static PIDFCoefficients shooterCoefficients = new PIDFCoefficients(0.004, 0, 0, 0.0005);
    private final PIDFController shooterController = new PIDFController(shooterCoefficients);
    private double shooterPower;
    private boolean fixJam;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private CRServo agitator;
    private ShaftDirection feederDirection = ShaftDirection.STOP;
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
    private LED yellowLed;
    private LED greenLed;
    private final ElapsedTime ledTimer = new ElapsedTime();
    public GoBildaPinpointDriver odo;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Pose poseFromCamera;

    /** Position:
        * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
        * inches above the ground - you would need to set the position to (-5, 7, 12).
        *
        * Orientation:
        * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
        * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
        * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
        * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
        0.25, 7.5, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
        0, -90 + 18, 0, 0);

    public void initialize(HardwareMap hardwareMap) {
        agitator = hardwareMap.get(CRServo.class, "agitator");
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
        yellowLed = hardwareMap.get(LED.class, "yellowLed");
        greenLed = hardwareMap.get(LED.class, "greenLed");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setDirection(CRServo.Direction.REVERSE);

        liftTargetPosition = rightLift.getCurrentPosition();
        rightLift.setTargetPosition(liftTargetPosition);
        leftLift.setTargetPosition(liftTargetPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        redLed.off();
        yellowLed.off();
        greenLed.off();
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
    public void runAuton() {
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
        if (fastest > MAX_WHEEL_SPEED) {
            double scaleFactor = fastest / MAX_WHEEL_SPEED;
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
        if(intakeDirection == ShaftDirection.IN) {
            intake.setPower(0.7);
            agitator.setPower(-1);
        } else if(intakeDirection == ShaftDirection.OUT) {
            intake.setPower(-1);
            agitator.setPower(0);
        } else {
            intake.setPower(0);
            agitator.setPower(0);
        }
    }

    private void runShooter() {
        if(fixJam) {
         shooter.setPower(-1);
         return;
        }
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
        leftLift.setVelocity(LIFT_VELOCITY);
        rightLift.setVelocity(LIFT_VELOCITY);
    }
    private void runFeeder() {
        if(fixJam) {
            rightFeeder.setPower(-1);
            leftFeeder.setPower(-1);
            return;
        }
        if(feederDirection == ShaftDirection.IN) {
            rightFeeder.setPower(1);
            leftFeeder.setPower(1);
        } else if(feederDirection == ShaftDirection.OUT) {
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
        } else if (Math.abs(shooter.getVelocity() - shooterTarget) < 100) {
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
        yellowLed.enable(yellow);

        greenLed.enable(poseFromCamera != null);
    }
    //Setters
    public void setLiftTargetPosition(int liftTargetPosition) {
        this.liftTargetPosition = liftTargetPosition;
    }
    public int getLiftTargetPosition() {
        return liftTargetPosition;
    }

    public void setJamFix(boolean fix) {
        fixJam = fix;
    }

    public void setIntakeDirection(ShaftDirection intakeDirection) {
        this.intakeDirection = intakeDirection;
    }
    public void setShooterTarget(double shooterTarget) {
        this.shooterTarget = shooterTarget;
    }
    public void setFeederDirection(ShaftDirection feederDirection) {
        this.feederDirection = feederDirection;
    }
    public void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

            // The following default settings are available to un-comment and edit as needed.
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setCameraPose(cameraPosition, cameraOrientation)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            .setLensIntrinsics(935.184, 935.184, 316.249, 254.729)

            // ... these parameters are fx, fy, cx, cy.

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);


        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


    }   // end method initAprilTag()
    public void runAprilTags() {
        poseFromCamera = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                poseFromCamera = new Pose(
                    -detection.robotPose.getPosition().x,
                    -detection.robotPose.getPosition().y,
                    MathFunctions.normalizeAngle(detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI/2)
                );
            }
        }
    }

    @SuppressLint("DefaultLocale")
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
        telemetry.addData("feeder",feederDirection);
        telemetry.addData("intake",intakeDirection);
        telemetry.update();
    }
    
    public void addPedroPathingTelemetry(Telemetry telemetry, Telemetry dashboardTelemetry, Follower follower) {
        boolean hasPath = follower.getCurrentPath() != null;
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        dashboardTelemetry.addData("hasPath", hasPath);
        dashboardTelemetry.addData("followerBusy", follower.isBusy());
        dashboardTelemetry.addData("atParametricEnd", hasPath && follower.atParametricEnd());
        dashboardTelemetry.addData("distanceTraveledOnPath", hasPath ? follower.getDistanceTraveledOnPath() : 0);
        dashboardTelemetry.addData("pathCompletion", hasPath ? follower.getPathCompletion() : 0);
        dashboardTelemetry.addData("distanceRemaining", hasPath ? follower.getDistanceRemaining() : 0);
        dashboardTelemetry.addData("closestX", hasPath ? follower.getClosestPose().getPose().getX() : 0);
        dashboardTelemetry.addData("closestY", hasPath ? follower.getClosestPose().getPose().getY() : 0);
        dashboardTelemetry.addData("distanceClosest", hasPath ? follower.getPose().distanceFrom(follower.getClosestPose().getPose()) : 0);
        dashboardTelemetry.addData("velocityM", follower.getVelocity().getMagnitude());
        dashboardTelemetry.addData("velocityT", follower.getVelocity().getTheta());
        dashboardTelemetry.addData("accelerationM", follower.getAcceleration().getTheta());
        dashboardTelemetry.addData("accelerationT", follower.getAcceleration().getTheta());
        dashboardTelemetry.addData("driveError", follower.getDriveError());
        dashboardTelemetry.addData("headingError", follower.getHeadingError());
        dashboardTelemetry.addData("translationalErrorM", hasPath ? follower.getTranslationalError().getMagnitude() : 0);
        dashboardTelemetry.addData("translationalErrorT", hasPath ? follower.getTranslationalError().getTheta() : 0);
        telemetry.addData("xCam", poseFromCamera != null ? poseFromCamera.getX() : 0);
        telemetry.addData("yCam", poseFromCamera != null ? poseFromCamera.getY() : 0);
        telemetry.addData("headingCam", poseFromCamera != null ? Math.toDegrees(poseFromCamera.getHeading()) : 0);
    }
    public static Pose mirror(Pose p) {
        return new Pose(p.getX(), -p.getY(), -p.getHeading());
    }
}
