package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.dashboard.DashboardTelemetry;

@TeleOp
@Config
public class DriverControl extends LinearOpMode {
    private RobotCommon common;
    private final DashboardTelemetry dashboardTelemetry = DashboardTelemetry.getInstance();
    public static double ROBOT_SLOW = 1200;
    public static double ROBOT_FAST = 2000;
    public static int LIFT_UP = 1;
    public static double SHOOTER_X = 1300;
    public static double SHOOTER_Y = 1400;
    public static double SHOOTER_START = 1600;
    private int headingOffset = 0;
    
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                common.run();
                common.odo.update();
                common.sendTelemetry(telemetry);
            }
        }
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        common = new RobotCommon();
        common.initialize(hardwareMap);
        common.odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        common.sendTelemetry(telemetry);
        if (blackboard.containsKey("headingOffset")) {
            headingOffset = (int)blackboard.get("headingOffset");
        }
    }

    private void controls() {
        if (gamepad2.a) {
            common.setIntakeDirection(RobotCommon.ShaftDirection.IN);
        } else if (gamepad2.b) {
            common.setIntakeDirection(RobotCommon.ShaftDirection.OUT);
        } else {
            common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
        }
        if (gamepad1.dpad_up) {
            common.setLiftTargetPosition(LIFT_UP);
        }
        if (gamepad2.y) {
            common.setShooterTarget(SHOOTER_Y);
        } else if (gamepad2.x) {
            common.setShooterTarget(SHOOTER_X);
        } else if (gamepad2.back) {
            common.setShooterTarget(SHOOTER_START);
        } else if (gamepad2.guide) {
            common.setShooterTarget(0);
        }

        common.setJamFix(gamepad2.dpad_down);

        if (gamepad2.right_bumper) {
            common.setFeederDirection(RobotCommon.ShaftDirection.IN);
        } else if (gamepad2.left_bumper) {
            common.setFeederDirection(RobotCommon.ShaftDirection.OUT);
        } else {
            common.setFeederDirection(RobotCommon.ShaftDirection.STOP);
        }
        double speed = ROBOT_FAST;
        if (gamepad1.a) {
            speed = ROBOT_SLOW;
        }

        double x = square(-gamepad1.left_stick_y) * speed;
        double y = square(gamepad1.left_stick_x) * speed;
        double heading = common.odo.getHeading(AngleUnit.RADIANS) - headingOffset;
        double vx = x * Math.cos(heading) - y * Math.sin(heading);
        double vy = x * Math.sin(heading) + y * Math.cos(heading);
        if (gamepad1.guide)  {
            common.odo.resetPosAndIMU();
            headingOffset = 0;
            blackboard.put("headingOffset", 0);
        }
        double rot = square(gamepad1.right_trigger-gamepad1.left_trigger) * ROBOT_SLOW;

        common.setRobotSpeed(vx, vy, rot);
    }
    public static double square(double amount) {
        return amount * Math.abs(amount);
    }

}

