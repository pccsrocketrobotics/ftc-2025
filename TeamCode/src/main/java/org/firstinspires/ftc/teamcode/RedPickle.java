package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "Pick up from loading zone and shoot from small tri")
@Config
public class RedPickle extends BluePickle{

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        farShotPose = RobotCommon.mirror(farShotPose);
        loadingPose1 = RobotCommon.mirror(loadingPose1);
        loadingPose2 = RobotCommon.mirror(loadingPose2);
        endPose = RobotCommon.mirror(endPose);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }

}