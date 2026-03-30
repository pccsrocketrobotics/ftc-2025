package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "9 ball aligned with puzzle piece by small tri shoots from small tri")
@Config
public class RedWallFar9 extends BlueWallFar9 {

    @Override
    public void initialize() {
        startingPose = PoseUtil.mirror(startingPose);
        farShotPose = PoseUtil.mirror(farShotPose);
        alignPose3 = PoseUtil.mirror(alignPose3);
        pickupPose3 = PoseUtil.mirror(pickupPose3);
        alignPose2 = PoseUtil.mirror(alignPose2);
        pickupPose2 = PoseUtil.mirror(pickupPose2);
        endPose = PoseUtil.mirror(endPose);
        super.initialize();
    }

    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }
}
