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
    protected Pose loadingPose1 = new Pose(-58,64,Math.toRadians(170));
    protected Pose loadingPose2 = new Pose(-58,62,Math.toRadians(170));
    protected Pose loadingFace = new Pose(-72, 64, Math.toRadians(0));
    protected Pose loadingControl1Pose = new Pose(-54, 17);
    protected Pose loadingControl2Pose =  new Pose(-40, 61);
    protected Pose shotControlPose = new Pose(-52, 50);
    protected Pose endPose = new Pose(-60,60,Math.toRadians(90));
    public static double SHOOTER_AUTON = 1540;
    public static double FEEDER_TIME = 1000;
    public static double SHOOTING_TIME = 200;
    public static double START_DELAY = 1500;
    public static double PICKUP_TIME1 = 4000;
    public static double PICKUP_TIME2 = 3000;
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
                    loadingControl1Pose,
                    loadingControl2Pose,
                    loadingPose1
                ))
                .setHeadingInterpolation(RobotCommon.facingPoint(loadingFace.getX(), loadingFace.getY()))
//                .setHeadingInterpolation(HeadingInterpolator.piecewise(
//                    new HeadingInterpolator.PiecewiseNode(0, 0.5, HeadingInterpolator.tangent),
//                    new HeadingInterpolator.PiecewiseNode(0.5, 1.0, HeadingInterpolator.constant(loadingPose1.getHeading()))
//                ))
                .build();
        PathChain shootingPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    loadingPose1,
                    shotControlPose,
                    farShotPose
                ))
                .setConstantHeadingInterpolation(farShotPose.getHeading())
                .build();
        PathChain loadingZonePath2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    farShotPose,
                    loadingControl1Pose,
                    loadingControl2Pose,
                    loadingPose2
                ))
                .setHeadingInterpolation(RobotCommon.facingPoint(loadingFace.getX(), loadingFace.getY()))
                .build();
        PathChain shootingPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    loadingPose2,
                    shotControlPose,
                    farShotPose
                ))
                .setConstantHeadingInterpolation(farShotPose.getHeading())
                .build();
        PathChain endPath = follower.pathBuilder()
                .addPath(new BezierLine(farShotPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
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
                        if (!follower.isBusy() && stateTime.milliseconds() > START_DELAY) {
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
                        if (stateTime.milliseconds() > PICKUP_TIME1) {
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
                        if (!follower.isBusy() || stateTime.milliseconds() > PICKUP_TIME2) {
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
                        break;
                    case 24:
                        if (!follower.isBusy()) {
                            //common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
                            changeState(25);
                        }
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
