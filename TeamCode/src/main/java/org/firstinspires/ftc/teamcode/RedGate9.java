package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(preselectTeleOp = "DriverControl", group = "6 ball corner against goal")
@Config
public class RedGate9 extends BlueGate9{

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        halfShotPose = RobotCommon.mirror(halfShotPose);
        alignPose1 = RobotCommon.mirror(alignPose1);
        pickupPose1 = RobotCommon.mirror(pickupPose1);
        alignPose2 = RobotCommon.mirror(alignPose2);
        pickupPose2 = RobotCommon.mirror(pickupPose2);
        halfShotPose2 = RobotCommon.mirror(halfShotPose2);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", 90);
    }

}