package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "DriverControlAssist", group = "Pick up from loading zone and shoot from small tri")
@Config
public class RedPickle extends BluePickle {

    @Override
    public void initialize() {
        startingPose = PoseUtil.mirror(startingPose);
        farShotPose = PoseUtil.mirror(farShotPose);
        loadingPose1 = PoseUtil.mirror(loadingPose1);
        loadingPose2 = PoseUtil.mirror(loadingPose2);
        loadingControl1Pose = PoseUtil.mirror(loadingControl1Pose);
        loadingControl2Pose = PoseUtil.mirror(loadingControl2Pose);
        slamPose1 = PoseUtil.mirror(slamPose1);
        slamBackPose = PoseUtil.mirror(slamBackPose);
        slamPose2 = PoseUtil.mirror(slamPose2);
        shotControlPose = PoseUtil.mirror(shotControlPose);
        loadingFace = PoseUtil.mirror(loadingFace);
        endPose = PoseUtil.mirror(endPose);
        super.initialize();
    }

    @Override
    protected void setBlackboard() {
        blackboard.put("headingOffset", -90);
    }
}