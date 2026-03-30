package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "6 ball aligned with puzzle piece by small tri shoots from small tri")
@Config
public class RedWallFar extends BlueWallFar {

    @Override
    public void initialize() {
        startingPose = PoseUtil.mirror(startingPose);
        farShotPose = PoseUtil.mirror(farShotPose);
        alignPose3 = PoseUtil.mirror(alignPose3);
        pickupPose3 = PoseUtil.mirror(pickupPose3);
        super.initialize();
    }

    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }
}
