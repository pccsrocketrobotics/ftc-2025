package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final LED redLed;
    private final LED blueLed;
    private final LED yellowLed;
    private final LED greenLed;
    private final Shooter shooter;
    private final ColorSensor colorSensor;
    private final ElapsedTime ledTimer = new ElapsedTime();

    public LEDs(HardwareMap hardwareMap, Shooter shooter, ColorSensor colorSensor) {
        this.shooter = shooter;
        this.colorSensor = colorSensor;
        redLed = hardwareMap.get(LED.class, "redLed");
        blueLed = hardwareMap.get(LED.class, "blueLed");
        yellowLed = hardwareMap.get(LED.class, "yellowLed");
        greenLed = hardwareMap.get(LED.class, "greenLed");

        redLed.off();
        yellowLed.off();
        greenLed.off();
        blueLed.off();
    }

    @Override
    public void periodic() {
        boolean red = false;
        boolean blue = false;
        boolean yellow = false;
        boolean green = false;

        if (shooter.getVelocity() < 100 && shooter.getTarget() == 0) {
            ledTimer.reset();
            red = true;
        } else if (Math.abs(shooter.getVelocity() - shooter.getTarget()) < 100) {
            if (ledTimer.milliseconds() > 200) {
                yellow = true;
            } else {
                red = true;
            }
        } else {
            ledTimer.reset();
            yellow = true;
        }
        green = colorSensor.hasBall();

        // NOTE: only update the LEDs if the state changes to avoid multiple hardware writes
        redLed.enable(red);
        blueLed.enable(blue);
        yellowLed.enable(yellow);
        greenLed.enable(green);
    }
}
