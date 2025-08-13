package org.firstinspires.ftc.teamcode.common.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class Methods {
    public static boolean checkButton(Gamepad gamepad, String button) {
        try {
            String buttons = String.valueOf(gamepad).substring(75).substring(1);
            return !buttons.contains(button);
        }
        catch (Exception ignored) {
            return (true);
        }
    }

    public static double round(double number, int places) {
        return new BigDecimal((String.valueOf(number))).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }
}
