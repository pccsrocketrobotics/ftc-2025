package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "Pick up from loading zone and shoot from small tri")
@Config
public class RedPickle extends BluePickle{

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        farShotPose = RobotCommon.mirror(farShotPose);
        loadingPose1 = RobotCommon.mirror(loadingPose1);
        loadingPose2 = RobotCommon.mirror(loadingPose2);
        loadingControl1Pose = RobotCommon.mirror(loadingControl1Pose);
        loadingControl2Pose = RobotCommon.mirror(loadingControl2Pose);
        slamPose1=RobotCommon.mirror(slamPose1);
        slamBackPose= RobotCommon.mirror(slamBackPose);
        slamPose2=RobotCommon.mirror(slamPose2);
        shotControlPose = RobotCommon.mirror(shotControlPose);
        loadingFace = RobotCommon.mirror(loadingFace);
        endPose = RobotCommon.mirror(endPose);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }

}