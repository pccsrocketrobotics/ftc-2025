package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAtomic")
@Config
public class RedCenterLine extends BlueCenterLine {

    @Override
    public void runOpMode() {
        super.runOpMode();
    }
    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", 0);
        blackboard.put("isBlueAlliance", false);
    }

}
