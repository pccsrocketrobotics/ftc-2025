package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "6 ball aligned with puzzle piece by small tri shoots from big tri")
@Config
public class RedWallMid extends BlueWallMid {

    @Override
    public void initialize() {
        startingPose = PoseUtil.mirror(startingPose);
        midShotPose = PoseUtil.mirror(midShotPose);
        alignPose2 = PoseUtil.mirror(alignPose2);
        pickupPose2 = PoseUtil.mirror(pickupPose2);
        super.initialize();
    }

    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }
}
