package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class LineTest extends OpMode {
    public static double DISTANCE = 40;
    public static boolean ZERO_DRIVE_AT_PARAMETRIC_END = false;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;
    private Telemetry dashTelemetry;
    static TelemetryManager telemetryM;
    public static Follower follower;
    private ElapsedTime timer;


    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        PanelsConfigurables.INSTANCE.refreshClass(this);
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();
        timer = new ElapsedTime();
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        forwards = new Path(new BezierLine(new Pose(0,0), new Pose(DISTANCE,0)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE,0), new Pose(0,0)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        boolean running = false;
        follower.errorCalculator.zeroDriveAtParametricEnd = ZERO_DRIVE_AT_PARAMETRIC_END;

        if (!follower.isBusy()) {
            if (timer.seconds() >= 1) {
                if (forward) {
                    forward = false;
                    follower.followPath(backwards);
                } else {
                    forward = true;
                    follower.followPath(forwards);
                }
            }
        } else {
            running = true;
            timer.reset();
        }

        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);

        dashTelemetry.addData("running", running);
        dashTelemetry.addData("forward", forward);
        dashTelemetry.addData("isBusy", follower.isBusy());
        dashTelemetry.addData("isHolding", follower.isHolding());
        dashTelemetry.addData("atParametricEnd", follower.atParametricEnd());
        dashTelemetry.addData("distanceTraveledOnPath", follower.getDistanceTraveledOnPath());
        dashTelemetry.addData("pathCompletion", follower.getPathCompletion());
        dashTelemetry.addData("distanceRemaining", follower.getDistanceRemaining());
        dashTelemetry.addData("tValue", follower.getCurrentTValue());
        dashTelemetry.addData("x", follower.getPose().getX());
        dashTelemetry.addData("y", follower.getPose().getY());
        dashTelemetry.addData("h", follower.getPose().getHeading());
        dashTelemetry.addData("closestX", follower.getClosestPose().getPose().getX());
        dashTelemetry.addData("closestY", follower.getClosestPose().getPose().getY());
        dashTelemetry.addData("distanceClosest", follower.getPose().distanceFrom(follower.getClosestPose().getPose()));
//        dashTelemetry.addData("distanceToGoal", follower.errorCalculator.getDistanceToGoal());

        dashTelemetry.addData("velocityM", follower.getVelocity().getMagnitude());
        dashTelemetry.addData("velocityT", follower.getVelocity().getTheta());
        dashTelemetry.addData("accelerationM", follower.getAcceleration().getTheta());
        dashTelemetry.addData("accelerationT", follower.getAcceleration().getTheta());

        dashTelemetry.addData("driveError", follower.getDriveError());
        dashTelemetry.addData("headingError", follower.getHeadingError());
        dashTelemetry.addData("translationalErrorM", follower.getTranslationalError().getMagnitude());
        dashTelemetry.addData("translationalErrorT", follower.getTranslationalError().getTheta());

        dashTelemetry.addData("driveVectorM", follower.getDriveVector().getMagnitude());
        dashTelemetry.addData("driveVectorT", follower.getDriveVector().getTheta());
        dashTelemetry.addData("headingVectorM", follower.getHeadingVector().getMagnitude());
        dashTelemetry.addData("headingVectorT", follower.getHeadingVector().getTheta());
        dashTelemetry.addData("translationalCorrectionM", follower.getTranslationalCorrection().getMagnitude());
        dashTelemetry.addData("translationalCorrectionT", follower.getTranslationalCorrection().getTheta());
        dashTelemetry.addData("centripetalForceCorrectionM", follower.getCentripetalForceCorrection().getMagnitude());
        dashTelemetry.addData("centripetalForceCorrectionT", follower.getCentripetalForceCorrection().getTheta());
        dashTelemetry.addData("correctiveVectorM", follower.vectorCalculator.getCorrectiveVector().getMagnitude());
        dashTelemetry.addData("correctiveVectorT", follower.vectorCalculator.getCorrectiveVector().getTheta());
        dashTelemetry.update();

    }
}