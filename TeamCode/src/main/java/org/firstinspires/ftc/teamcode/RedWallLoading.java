package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "Gets loading zone and last set of balls")
@Config
public class RedWallLoading extends BlueWallLoading {

    @Override
    public void initialize() {
        startingPose = PoseUtil.mirror(startingPose);
        farShotPose = PoseUtil.mirror(farShotPose);
        alignPose3 = PoseUtil.mirror(alignPose3);
        pickupPose3 = PoseUtil.mirror(pickupPose3);
        alignPose2 = PoseUtil.mirror(alignPose2);
        pickupPose2 = PoseUtil.mirror(pickupPose2);
        endPose = PoseUtil.mirror(endPose);
        slamBackPose = PoseUtil.mirror(slamBackPose);
        slamPose1 = PoseUtil.mirror(slamPose1);
        slamPose2 = PoseUtil.mirror(slamPose2);
        super.initialize();
    }

    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }
}

