package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DriverControl extends LinearOpMode {
    private RobotCommon common;
    public static double ROBOT_SPEED = 1500;

    private DcMotorEx intake;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                common.run();
                if (common.isCompetitionBot()) {
                    common.odo.update();
                }
                common.sendTelemetry(telemetry);
            }
        }
    }

    private void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common = new RobotCommon();
        common.initialize(hardwareMap);
        if (common.isCompetitionBot()) {
            common.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        }

        intake = hardwareMap.get(DcMotorEx.class, "arm");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void controls() {
        double x = square(-gamepad1.left_stick_y) * ROBOT_SPEED;
        double y = square(gamepad1.left_stick_x) * ROBOT_SPEED;
        double heading = common.isCompetitionBot() ? common.odo.getHeading(AngleUnit.RADIANS) : 0;
        double vx = x * Math.cos(heading) - y * Math.sin(heading);
        double vy = x * Math.sin(heading) + y * Math.cos(heading);
        if (gamepad1.guide && common.isCompetitionBot())  {
            common.odo.resetPosAndIMU();
        }
        double rot = square(gamepad1.right_trigger-gamepad1.left_trigger) * ROBOT_SPEED;

        common.setRobotSpeed(vx, vy, rot);

        if (gamepad1.a) {
            intake.setPower(1);
        } else if (gamepad1.b) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }

    public static double square(double amount) {
        return amount * Math.abs(amount);

    }
}

