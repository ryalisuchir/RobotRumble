package org.firstinspires.ftc.teamcode.opmode.Testing;

import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.RED;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

import java.util.Set;

@Disabled
@TeleOp(name = "ColorSensorHSV", group = "Sensor")
public class ColorSensorHSV extends LinearOpMode {

    RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        float[] hsv = new float[3];

        while (opModeIsActive()) {
            // Read RGB from color sensor
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();

            // Normalize and convert to HSV
            Color.RGBToHSV(r * 8, g * 8, b * 8, hsv); // Scale to 0–255

            float hue = hsv[0];
            float sat = hsv[1];
            float val = hsv[2];

            ExtendAndSpinCommand.SampleColorDetected colorDetected = detectSampleColor(hue, sat, val);

            if (Set.of(RED).contains(colorDetected)) {
                telemetry.addLine("Hooray");
            }

            telemetry.addData("R", r);
            telemetry.addData("G", g);
            telemetry.addData("B", b);
            telemetry.addData("Hue", hue);
            telemetry.addData("Sat", sat);
            telemetry.addData("Val", val);
            telemetry.addData("Detected Color", colorDetected);
            telemetry.addData("Distance", colorSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            sleep(100);
        }
    }

    // Helper function to determine the color based on HSV ranges
    public static ExtendAndSpinCommand.SampleColorDetected detectSampleColor(float hue, float sat, float val) {
        if (val < 0.2) return ExtendAndSpinCommand.SampleColorDetected.NONE;

        if ((hue >= 0 && hue <= 25) || (hue >= 330 && hue <= 360)) {
            if (sat > 0.4 && val > 0.2) return RED;
        }
        if (hue >= 200 && hue <= 250) {
            if (sat > 0.4 && val > 0.2) return ExtendAndSpinCommand.SampleColorDetected.BLUE;
        }
        if (hue >= 50 && hue <= 100) {
            if (sat > 0.7 && val > 0.3) return ExtendAndSpinCommand.SampleColorDetected.YELLOW;
        }

        return ExtendAndSpinCommand.SampleColorDetected.NONE;
    }
}
