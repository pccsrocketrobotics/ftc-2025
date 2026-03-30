package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class ColorSensor extends SubsystemBase {
    public static float COLOR_GAIN = 2;

    private final NormalizedColorSensor colorSensor;
    private double ballDistance;

    public ColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        colorSensor.setGain(COLOR_GAIN);
    }

    @Override
    public void periodic() {
        ballDistance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }

    public double getBallDistance() {
        return ballDistance;
    }

    public boolean hasBall() {
        return ballDistance < 8;
    }
}
