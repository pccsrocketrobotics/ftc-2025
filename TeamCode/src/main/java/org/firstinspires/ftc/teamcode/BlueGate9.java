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

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "9 ball corner against goal")
@Config
public class BlueGate9 extends LinearOpMode {
    private Follower follower;
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    private RobotCommon common;
    protected Pose startingPose = new Pose(48.1,50.5,Math.toRadians(0));
    protected Pose halfShotPose = new Pose(27.5,27,Math.toRadians(45));
    protected Pose alignPose1 = new Pose(12.7,26.8,Math.toRadians(90));
    protected Pose pickupPose1 = new Pose(12.7,50,Math.toRadians(90));
    protected Pose alignPose2 = new Pose(-13.6,26.3,Math.toRadians(90));
    protected Pose pickupPose2 = new Pose(-12.7,50,Math.toRadians(90));
    protected Pose halfShotPose2 = new Pose(34,20.5,Math.toRadians(54));
    public static double SHOOTER_AUTON = 1325;
    public static double FEEDER_TIME = 1000;
    public static double SHOOTING_TIME = 200;
    private int shots = 0;
    private int state = 0;
    private final ElapsedTime stateTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initialize();

        PathChain shootingPath = follower.pathBuilder()
            .addPath(new BezierLine(startingPose,halfShotPose))
            .setLinearHeadingInterpolation(startingPose.getHeading(),halfShotPose.getHeading())
            .build();
        PathChain ballAlignPath = follower.pathBuilder()
            .addPath(new BezierLine(halfShotPose, alignPose1))
            .setLinearHeadingInterpolation(halfShotPose.getHeading(), alignPose1.getHeading())
            .build();
        PathChain ballPickupPath = follower.pathBuilder()
            .addPath(new BezierLine(alignPose1, pickupPose1))
            .build();
        PathChain shootingPath2 = follower.pathBuilder()
            .addPath(new BezierLine(pickupPose1, halfShotPose))
            .setLinearHeadingInterpolation(pickupPose1.getHeading(), halfShotPose.getHeading())
            .build();
        PathChain ballAlignPath2 = follower.pathBuilder()
            .addPath(new BezierLine(halfShotPose, alignPose2))
            .setLinearHeadingInterpolation(halfShotPose.getHeading(), alignPose2.getHeading())
            .build();
        PathChain ballPickupPath2 = follower.pathBuilder()
            .addPath(new BezierLine(alignPose2, pickupPose2))
            .build();
        PathChain shootingPath3 = follower.pathBuilder()
            .addPath(new BezierLine(pickupPose2, halfShotPose2))
            .setLinearHeadingInterpolation(pickupPose1.getHeading(), halfShotPose2.getHeading())
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
                        follower.followPath(ballAlignPath);
                        changeState(6);
                        break;
                    case 6:
                        if (!follower.isBusy()) {
                            changeState(7);
                        }
                        break;
                    case 7:
                        follower.followPath(ballPickupPath);
                        changeState(8);
                        break;
                    case 8:
                        if (!follower.isBusy()) {
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
                        if (stateTime.milliseconds() > SHOOTING_TIME) {
                            shots++;
                            if (shots < 3) {
                                changeState(11);
                            } else {
                                changeState(14);
                            }
                        }
                        break;
                    case 14:
                        follower.followPath(ballAlignPath2);
                        changeState(15);
                        break;
                    case 15:
                        if (!follower.isBusy()) {
                            changeState(16);
                        }
                        break;
                    case 16:
                        follower.followPath(ballPickupPath2);
                        changeState(17);
                        break;
                    case 17:
                        if (!follower.isBusy()) {
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
                        if (stateTime.milliseconds() > SHOOTING_TIME) {
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
                        changeState(24);
                        common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
                        break;
                }

                follower.update();
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

