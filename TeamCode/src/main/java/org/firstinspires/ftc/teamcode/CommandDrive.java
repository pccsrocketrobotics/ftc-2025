package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.BooleanSupplier;

@TeleOp
public class CommandDrive extends CommandOpMode {
    @Override
    public void initialize() {
        // 1. Create subsystems
        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // 2. Register them so periodic() runs
        register(drive, intake);

        // 3. Wrap the gamepads
        GamepadEx operator = new GamepadEx(gamepad1);

        // 4. Bind buttons to commands
        drive.setDefaultCommand(
                drive.go(
                        () -> -gamepad1.left_stick_y,
                        () -> gamepad1.left_stick_x,
                        () -> gamepad1.right_trigger - gamepad1.left_trigger
                )
        );
        intake.setDefaultCommand(intake.inCommand());


        operator.getGamepadButton(GamepadKeys.Button.B).whenHeld(intake.outCommand());
        operator.getGamepadButton(GamepadKeys.Button.Y).whenHeld(intake.stopCommand());
        operator.getGamepadButton(GamepadKeys.Button.X).whenHeld(intake.stopCommand());

        operator.getGamepadButton(GamepadKeys.Button.BACK).toggleWhenPressed(
                new StartEndCommand(
                        () -> drive.setSpeedMult(1),
                        () -> drive.setSpeedMult(Drive.SLOW_SPEED)

                )
        );
    }
}
