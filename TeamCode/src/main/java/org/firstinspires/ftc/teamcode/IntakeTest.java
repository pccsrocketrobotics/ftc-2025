package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
@Config
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "arm");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        int intakePower = 50;
        Object powerBB = blackboard.get("intakePower");
        if (powerBB != null) {
            intakePower = (int) powerBB;
        }

        waitForStart();

        while (!isStopRequested()) {
            boolean intakePowerChanged = false;
            if (gamepad1.dpadUpWasPressed()) {
                intakePower -= 5;
                intakePowerChanged = true;
            }
            if (gamepad1.dpadDownWasPressed()) {
                intakePower += 5;
                intakePowerChanged = true;
            }
            if (intakePowerChanged) {
                intakePower = Range.clip(intakePower, 0, 100);
                blackboard.put("intakePower", intakePower);
            }

            if (gamepad1.a) {
                intake.setPower(intakePower / 100.0);
            } else if (gamepad1.b) {
                intake.setPower(-intakePower / 100.0);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("intakePower", intakePower);
            telemetry.update();
        }
    }
}
