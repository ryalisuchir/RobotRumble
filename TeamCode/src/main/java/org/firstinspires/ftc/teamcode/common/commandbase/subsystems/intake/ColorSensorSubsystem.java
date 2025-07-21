package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensorSubsystem extends SubsystemBase {
    public final NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;

    public ColorSensorSubsystem(NormalizedColorSensor colorSensorInput) {
        colorSensor = colorSensorInput;
    }

    public float[] getColor() {
        colors = colorSensor.getNormalizedColors();
        return new float[] { colors.red, colors.green, colors.blue };
    }

}
