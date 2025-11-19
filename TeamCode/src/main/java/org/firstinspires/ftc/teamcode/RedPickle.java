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
        launchAlignPose1 = RobotCommon.mirror(launchAlignPose1);
        launchPickupPose1 = RobotCommon.mirror(launchPickupPose1);
        launchAlignPose2 = RobotCommon.mirror(launchAlignPose2);
        launchPickupPose2 = RobotCommon.mirror(launchPickupPose2);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }

}