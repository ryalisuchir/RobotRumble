package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous
public class ColorSensor extends LinearOpMode {
    private NormalizedColorSensor test_color;

    @Override
    public void runOpMode() {
        test_color = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
            NormalizedRGBA colors = test_color.getNormalizedColors();

            //Determining the amount of red, green, and blue
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.update();
        }
    }
}