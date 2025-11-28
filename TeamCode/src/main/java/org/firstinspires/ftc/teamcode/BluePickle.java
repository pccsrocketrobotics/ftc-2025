package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.dashboard.DashboardTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.RobotDrawing;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "Pick up from loading zone and shoot from small tri")
@Config
public class BluePickle extends LinearOpMode{
    private Follower follower;
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();

    private RobotCommon common;
    protected Pose startingPose = new Pose(-60,16.8,Math.toRadians(0));
    protected Pose farShotPose = new Pose(-53.5,13.5,Math.toRadians(23.5));
    protected Pose loadingPose1 = new Pose(-62,62,Math.toRadians(180));
    protected Pose loadingPose2 = new Pose(-62,62,Math.toRadians(172));
    protected Pose endPose = new Pose(-60,46,Math.toRadians(90));
    public static double SHOOTER_AUTON = 1560;
    public static double FEEDER_TIME = 1000;
    public static double SHOOTING_TIME = 200;
    public static double START_DELAY = 1500;
    public static double PICKUP_TIME = 2000;
    private int shots = 0;
    private int state = 0;
    private final ElapsedTime stateTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initialize();

        PathChain shootingPath = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, farShotPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), farShotPose.getHeading())
                .build();
        PathChain loadingZonePath = follower.pathBuilder()
                .addPath(new BezierCurve(
                    farShotPose,
                    new Pose(-41, 22),
                    new Pose(-46, 67),
                    loadingPose1
                ))
                .build();
        PathChain shootingPath2 = follower.pathBuilder()
                .addPath(new BezierLine(loadingPose1, farShotPose))
                .setLinearHeadingInterpolation(loadingPose1.getHeading(), farShotPose.getHeading())
                .build();
        PathChain loadingZonePath2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    farShotPose,
                new Pose(-46,49),
                new Pose (-43,59),
                    loadingPose2
                ))
                .build();
        PathChain shootingPath3 = follower.pathBuilder()
                .addPath(new BezierLine(loadingPose2, farShotPose))
                .setLinearHeadingInterpolation(loadingPose2.getHeading(), farShotPose.getHeading())
                .build();
        PathChain endPath = follower.pathBuilder()
            .addPath(new BezierLine(farShotPose, endPose))
            .setLinearHeadingInterpolation(farShotPose.getHeading(), endPose.getHeading())
            .build();


        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                switch (state) {
                    case 0:
                        common.setIntakeDirection(RobotCommon.ShaftDirection.IN);
                        common.setShooterTarget(SHOOTER_AUTON);
                        follower.followPath(shootingPath);
                        changeState(1);
                        break;
                    case 1:
                        if(!follower.isBusy() && stateTime.milliseconds() > START_DELAY){
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
                        if (stateTime.milliseconds() > SHOOTING_TIME || shots >= 2) {
                            shots++;
                            if (shots < 3) {
                                changeState(2);
                            } else {
                                changeState(5);
                            }
                        }
                        break;
                    case 5:
                        follower.followPath(loadingZonePath);
                        changeState(6);
                        break;
                    case 6:
                        if (!follower.isBusy() || stateTime.milliseconds() > PICKUP_TIME) {
                            changeState(9);
                        }
                        break;
                    case 9:
                        shots = 0;
                        follower.followPath(shootingPath2);
                        changeState(10);
                        break;
                    case 10:
                        if (!follower.isBusy()) {
                            changeState(11);
                        }
                        break;
                    case 11:
                        common.setFeederDirection(RobotCommon.ShaftDirection.IN);
                        changeState(12);
                        break;
                    case 12:
                        if (stateTime.milliseconds() > FEEDER_TIME) {
                            common.setFeederDirection(RobotCommon.ShaftDirection.STOP);
                            changeState(13);
                        }
                        break;
                    case 13:
                        if (stateTime.milliseconds() > SHOOTING_TIME || shots >= 2) {
                            shots++;
                            if (shots < 3) {
                                changeState(11);
                            } else {
                                changeState(14);
                            }
                        }
                        break;
                    case 14:
                        follower.followPath(loadingZonePath2);
                        changeState(15);
                        break;
                    case 15:
                        if (!follower.isBusy() || stateTime.milliseconds() > PICKUP_TIME) {
                            changeState(18);
                        }
                        break;
                    case 18:
                        shots = 0;
                        follower.followPath(shootingPath3);
                        changeState(19);
                        break;
                    case 19:
                        if (!follower.isBusy()) {
                            changeState(20);
                        }
                        break;
                    case 20:
                        common.setFeederDirection(RobotCommon.ShaftDirection.IN);
                        changeState(21);
                        break;
                    case 21:
                        if (stateTime.milliseconds() > FEEDER_TIME) {
                            common.setFeederDirection(RobotCommon.ShaftDirection.STOP);
                            changeState(22);
                        }
                        break;
                    case 22:
                        if (stateTime.milliseconds() > SHOOTING_TIME || shots >= 2) {
                            shots++;
                            if (shots < 3) {
                                changeState(20);
                            } else {
                                changeState(23);
                            }
                        }
                        break;
                    case 23:
                        common.setShooterTarget(0);
                        follower.followPath(endPath);
                        changeState(24);
                        common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
                        break;
                }

                follower.update();
                common.runAuton();
                common.runAprilTags();
                common.correctPose(follower);
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
        common.initAprilTag(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        blackboard.put("follower", follower);
        sendTelemetry();
        setBlackboard();
    }
    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }

    private void sendTelemetry() {
        telemetry.addData("state",state);
        telemetry.addData("shots",shots);
        common.addPedroPathingTelemetry(telemetry, dashboardTelemetry, follower);
        RobotDrawing.draw(dashboardTelemetry.getCurrentPacket(), follower);
        common.sendTelemetry(telemetry);
    }
}
