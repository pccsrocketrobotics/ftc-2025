package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "9 ball corner against goal")
@Config
public class RedGate9 extends BlueGate9 {

    @Override
    public void initialize() {
        startingPose = PoseUtil.mirror(startingPose);
        halfShotPose = PoseUtil.mirror(halfShotPose);
        alignPose1 = PoseUtil.mirror(alignPose1);
        pickupPose1 = PoseUtil.mirror(pickupPose1);
        alignPose2 = PoseUtil.mirror(alignPose2);
        pickupPose2 = PoseUtil.mirror(pickupPose2);
        halfShotPose2 = PoseUtil.mirror(halfShotPose2);
        super.initialize();
    }

    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }
}