package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControl")
@Config
public class RedGate extends BlueGate{

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        halfShotPose = RobotCommon.mirror(halfShotPose);
        ballPickingPose = RobotCommon.mirror(ballPickingPose);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }

}
