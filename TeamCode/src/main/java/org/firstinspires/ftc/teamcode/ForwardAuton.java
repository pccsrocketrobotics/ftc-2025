package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(preselectTeleOp = "DriverControl")
@Config
public class ForwardAuton extends LinearOpMode {
    private RobotCommon common;
    public static double ROBOT_SPEED = 1500;
    public static double ROBOT_FAST = 3000;
    public static int LIFT_UP = 0;
    public static double SHOOTER_X = 500;
    public static double SHOOTER_Y = 1000;
    private double shooterVelocity = 0;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        if(opModeIsActive()) {
            common.setRobotSpeed(1500, 0, 0);
            common.run();
            sleep(1000);
            common.setRobotSpeed(0, 0, 0);
            common.run();
        }
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common = new RobotCommon();
        common.initialize(hardwareMap);
        common.odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
    }
}

