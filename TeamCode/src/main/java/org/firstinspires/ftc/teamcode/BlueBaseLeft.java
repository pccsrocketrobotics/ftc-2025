package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.DashboardTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(preselectTeleOp = "DriverControlAtomic")
@Config
public class BlueBaseLeft extends LinearOpMode {
    private Follower follower;
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    private RobotCommon common;
    protected Pose startingPose = new Pose(-70,6,Math.toRadians(-90));
    @Override
    public void runOpMode() {
        initialize();
    }
   private void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        blackboard.put("follower", follower);
        setBlackboard();
    }
    protected void setBlackboard() {
        blackboard.put("headingOffset", 0);
        blackboard.put("isBlueAlliance", true);
    }

}
