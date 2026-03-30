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

    private boolean oldRed = false;
    private boolean oldBlue = false;
    private boolean oldYellow = false;
    private boolean oldGreen = false;

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
        boolean green = colorSensor.hasBall();

        if (red != oldRed) {
            redLed.enable(red);
        }
        if (blue != oldBlue) {
            blueLed.enable(blue);
        }
        if (yellow != oldYellow) {
            yellowLed.enable(yellow);
        }
        if (green != oldGreen) {
            greenLed.enable(green);
        }
        oldRed = red;
        oldBlue = blue;
        oldYellow = yellow;
        oldGreen = green;
    }
}
