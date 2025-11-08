package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControl")
@Config
public class RedWall extends BlueWall{

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        shootingPose = RobotCommon.mirror(shootingPose);
        alignPose2 = RobotCommon.mirror(alignPose2);
        pickupPose2 = RobotCommon.mirror(pickupPose2);
        super.runOpMode();
    }
}
