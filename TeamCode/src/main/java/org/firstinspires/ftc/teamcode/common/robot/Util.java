package org.firstinspires.ftc.teamcode.common.robot;

public class Util {
    public static Globals.SampleColorDetected detectSampleColor(float hue, float sat, float val) {
        if (val < 0.2) return Globals.SampleColorDetected.NONE;

        if ((hue >= 0 && hue <= 25) || (hue >= 330 && hue <= 360)) {
            if (sat > 0.4 && val > 0.2) return Globals.SampleColorDetected.RED;
        }
        if (hue >= 200 && hue <= 250) {
            if (sat > 0.4 && val > 0.2) return Globals.SampleColorDetected.BLUE;
        }
        if (hue >= 50 && hue <= 100) {
            if (sat > 0.7 && val > 0.3) return Globals.SampleColorDetected.YELLOW;
        }

        return Globals.SampleColorDetected.NONE;
    }
}
