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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class LineTest extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;
    private Telemetry dashTelemetry;
    static TelemetryManager telemetryM;
    public static Follower follower;


    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        PanelsConfigurables.INSTANCE.refreshClass(this);
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();
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
        follower.activateAllPIDFs();
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

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Running!");
        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);

        dashTelemetry.addData("forward", forward);
        dashTelemetry.addData("isBusy", follower.isBusy());
        dashTelemetry.addData("x", follower.poseTracker.getPose().getX());
        dashTelemetry.addData("y", follower.poseTracker.getPose().getY());
        dashTelemetry.addData("h", follower.poseTracker.getPose().getHeading());
        dashTelemetry.addData("currentVelocityM", follower.poseTracker.getVelocity().getMagnitude());
        dashTelemetry.addData("currentVelocityT", follower.poseTracker.getVelocity().getTheta());
        dashTelemetry.addData("currentAccelerationM", follower.poseTracker.getAcceleration().getTheta());
        dashTelemetry.addData("currentAccelerationT", follower.poseTracker.getAcceleration().getTheta());

        dashTelemetry.addData("translationalErrorM", follower.errorCalculator.getTranslationalError().getMagnitude());
        dashTelemetry.addData("translationalErrorT", follower.errorCalculator.getTranslationalError().getTheta());
        dashTelemetry.addData("driveError", follower.errorCalculator.getDriveError());
        dashTelemetry.addData("headingError", follower.errorCalculator.getHeadingError());

        dashTelemetry.addData("headingVectorM", follower.vectorCalculator.getHeadingVector().getMagnitude());
        dashTelemetry.addData("headingVectorT", follower.vectorCalculator.getHeadingVector().getTheta());
        dashTelemetry.addData("translationalVectorM", follower.vectorCalculator.getTranslationalVector().getMagnitude());
        dashTelemetry.addData("translationalVectorT", follower.vectorCalculator.getTranslationalVector().getTheta());
        dashTelemetry.addData("centripetalVectorM", follower.vectorCalculator.getCentripetalVector().getMagnitude());
        dashTelemetry.addData("centripetalVectorT", follower.vectorCalculator.getCentripetalVector().getTheta());
        dashTelemetry.addData("correctiveVectorM", follower.vectorCalculator.getCorrectiveVector().getMagnitude());
        dashTelemetry.addData("correctiveVectorT", follower.vectorCalculator.getCorrectiveVector().getTheta());
        dashTelemetry.update();

    }
}