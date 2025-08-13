package org.firstinspires.ftc.teamcode.common.robot;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;

public class ColorUtils {
    /**
     * Detect sample color from HSV values (hue [0-360], saturation [0-1], value [0-1])
     */
    public static ExtendAndSpinCommand.SampleColorDetected detectSampleColor(float hue, float sat, float val) {
        if (val < 0.2) return ExtendAndSpinCommand.SampleColorDetected.NONE;

        if ((hue >= 0 && hue <= 25) || (hue >= 330 && hue <= 360)) {
            if (sat > 0.4 && val > 0.2) return ExtendAndSpinCommand.SampleColorDetected.RED;
        }
        if (hue >= 200 && hue <= 250) {
            if (sat > 0.4 && val > 0.2) return ExtendAndSpinCommand.SampleColorDetected.BLUE;
        }
        if (hue >= 50 && hue <= 100) {
            if (sat > 0.7 && val > 0.3) return ExtendAndSpinCommand.SampleColorDetected.YELLOW;
        }

        return ExtendAndSpinCommand.SampleColorDetected.NONE;
    }

    /**
     * Checks if detected color is correct sample:
     * Yellow OR alliance color (from Globals.allianceColor)
     */
    public static boolean isCorrectSampleColor(ExtendAndSpinCommand.SampleColorDetected detected) {
        if (detected == ExtendAndSpinCommand.SampleColorDetected.YELLOW) return true;

        switch (detected) {
            case RED:
                return Globals.allianceColor == Globals.AllianceColor.RED;
            case BLUE:
                return Globals.allianceColor == Globals.AllianceColor.BLUE;
            default:
                return false;
        }
    }
}