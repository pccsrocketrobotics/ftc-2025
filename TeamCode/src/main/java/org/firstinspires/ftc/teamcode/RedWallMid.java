package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControl")
@Config
public class RedWallMid extends BlueWallMid {

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        midShotPose = RobotCommon.mirror(midShotPose);
        alignPose2 = RobotCommon.mirror(alignPose2);
        pickupPose2 = RobotCommon.mirror(pickupPose2);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }
}
