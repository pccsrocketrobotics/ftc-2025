package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControl")
@Config
public class RedWallFar extends BlueWallFar {

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        farShotPose = RobotCommon.mirror(farShotPose);
        alignPose3 = RobotCommon.mirror(alignPose3);
        pickupPose3 = RobotCommon.mirror(pickupPose3);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }
}
