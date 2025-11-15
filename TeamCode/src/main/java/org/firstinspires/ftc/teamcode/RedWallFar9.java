package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "9 ball aligned with puzzle piece by small tri shoots from small tri")
@Config
public class RedWallFar9 extends BlueWallFar9 {

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        farShotPose = RobotCommon.mirror(farShotPose);
        alignPose3 = RobotCommon.mirror(alignPose3);
        pickupPose3 = RobotCommon.mirror(pickupPose3);
        alignPose2 = RobotCommon.mirror(alignPose2);
        pickupPose2 = RobotCommon.mirror(pickupPose2);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }
}
