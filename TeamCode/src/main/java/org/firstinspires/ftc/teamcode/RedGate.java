package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class RedGate extends BlueGate{

    @Override
    public void runOpMode() {
        startingPose = startingPose.withY(-startingPose.getY());
        halfShotPose = halfShotPose.withY(-halfShotPose.getY());
        ballPickingPose = ballPickingPose.withY(-ballPickingPose.getY());
        super.runOpMode();
    }
}
