package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAtomic")
@Config
public class RedBaseLeft extends BlueBaseLeft {

    @Override
    public void runOpMode() {
        startingPose = RobotCommon.mirror(startingPose);
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", 0);
        blackboard.put("isBlueAlliance", false);
    }

}
