package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "6 ball corner against goal")
@Config
public class RedGate extends BlueGate {

    @Override
    public void initialize() {
        startingPose = PoseUtil.mirror(startingPose);
        halfShotPose = PoseUtil.mirror(halfShotPose);
        alignPose1 = PoseUtil.mirror(alignPose1);
        pickupPose1 = PoseUtil.mirror(pickupPose1);
        endPose = PoseUtil.mirror(endPose);
        super.initialize();
    }

    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }
}
