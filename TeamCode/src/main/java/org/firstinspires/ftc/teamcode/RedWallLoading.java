package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "Gets loading zone and last set of balls")
@Config
public class RedWallLoading extends BlueWallLoading {

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        farShotPose = RobotCommon.mirror(farShotPose);
        alignPose3 = RobotCommon.mirror(alignPose3);
        pickupPose3 = RobotCommon.mirror(pickupPose3);
        alignPose2 = RobotCommon.mirror(alignPose2);
        pickupPose2 = RobotCommon.mirror(pickupPose2);
        endPose = RobotCommon.mirror(endPose);
        slamBackPose = RobotCommon.mirror(slamBackPose);
        slamPose1 = RobotCommon.mirror(slamPose1);
        slamPose2 = RobotCommon.mirror(slamPose2);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }
}

