package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "6 ball corner against goal")
@Config
public class RedGate extends BlueGate{

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        halfShotPose = RobotCommon.mirror(halfShotPose);
        alignPose1 = RobotCommon.mirror(alignPose1);
        pickupPose1 = RobotCommon.mirror(pickupPose1);
        endPose = RobotCommon.mirror(endPose);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }

}
