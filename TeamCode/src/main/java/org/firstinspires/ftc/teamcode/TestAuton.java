package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dashboard.DashboardTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.RobotDrawing;

@Autonomous
@Config
public class TestAuton extends LinearOpMode {
    private Follower follower;
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    private RobotCommon common;
    public static Pose startingPose = new Pose(48.1,50.5,Math.toRadians(0));
    public static Pose halfShotPose = new Pose(27.5,27,Math.toRadians(45));
    public static Pose ballPickingPose= new Pose(12.7,32.8,Math.toRadians(90));
    public static double SHOOTER_AUTON = 1225;
    public static double FEEDER_TIME = 1000;
    public static double SHOOTING_TIME = 2000;
    private int shots = 0;
    private int state = 0;
    private final ElapsedTime stateTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initialize();

        PathChain halfShotPath = follower.pathBuilder()
                .addPath(new BezierLine(startingPose,halfShotPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(),halfShotPose.getHeading())
                .build();
        PathChain ballPickPath = follower.pathBuilder()
                .addPath(new BezierLine(halfShotPose,ballPickingPose))
                .setLinearHeadingInterpolation(halfShotPose.getHeading(),ballPickingPose.getHeading())
                .build();

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                follower.update();
                switch (state) {
                    case 0:
                        common.setIntakeDirection(RobotCommon.ShaftDirection.IN);
                        common.setShooterTarget(SHOOTER_AUTON);
                        follower.followPath(halfShotPath);
                        changeState(1);
                        break;
                    case 1:
                        if(!follower.isBusy()){
                            changeState(2);
                        }
                        break;
                    case 2:
                        common.setFeederDirection(RobotCommon.ShaftDirection.IN);
                        changeState(3);
                        break;
                    case 3:
                        if (stateTime.milliseconds() > FEEDER_TIME) {
                            common.setFeederDirection(RobotCommon.ShaftDirection.STOP);
                            changeState(4);
                        }
                        break;
                    case 4:
                        if (stateTime.milliseconds() > SHOOTING_TIME) {
                            shots++;
                            if (shots < 3) {
                                changeState(2);
                            } else {
                                changeState(5);
                            }
                        }
                        break;
                    case 5:
                        common.setShooterTarget(0);
                        follower.followPath(ballPickPath);
                        changeState(6);
                        break;
                    case 6:
                        if (!follower.isBusy()) {
                            common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
                            changeState(7);
                        }
                        break;
                }
                common.runAuton();
                sendTelemetry();
            }
        }
    }
    private void changeState(int newState) {
        state = newState;
        stateTime.reset();
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        RobotDrawing.setDashboardTelemetry(FtcDashboard.getInstance().getTelemetry());
        common = new RobotCommon();
        common.initialize(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        sendTelemetry();
    }

    private void sendTelemetry() {
        telemetry.addData("state",state);
        telemetry.addData("shots",shots);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("followerBusy", follower.isBusy());
        RobotDrawing.draw(dashboardTelemetry.getCurrentPacket(), follower);
        common.sendTelemetry(telemetry);
    }
}
