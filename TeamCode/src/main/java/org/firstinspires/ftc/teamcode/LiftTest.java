package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LiftTest extends LinearOpMode {
    private DcMotorEx lift;
    public static int LIFT_VELOCITY = 5000;
    public static double LIFT_CONTROLS = -100;
    public static int LIFT_MAX = 4000;
    private double targetPosition;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setVelocity(LIFT_VELOCITY);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                targetPosition = lift.getTargetPosition() + gamepad1.left_stick_y  * LIFT_CONTROLS;
                if(targetPosition < 0) {
                    targetPosition = 0;
                }

                if(targetPosition > LIFT_MAX){
                    targetPosition = LIFT_MAX;
                }








                lift.setTargetPosition((int) targetPosition);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setVelocity(LIFT_VELOCITY);
            }
        }



    }
}
