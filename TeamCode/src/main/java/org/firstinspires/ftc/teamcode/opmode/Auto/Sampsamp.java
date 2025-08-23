package org.firstinspires.ftc.teamcode.opmode.Auto;
import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.BLUE;
import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.YELLOW;
import android.graphics.Color;
import android.util.Log;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.BucketInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Collections;
import java.util.Set;

@Autonomous
public class Sampsamp extends OpMode {
    Robot robot;
    TrajectoryActionBuilder movement1;
    MecanumDrive.CancelableAction movement1a;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, Globals.BLUE_SIDEWAYS_START_POSE, true, true);

        telemetry.addLine("Initialized.");
        telemetry.update();

        movement1 = robot.driveSubsystem.trajectoryActionBuilderCorrection(Globals.BLUE_SIDEWAYS_START_POSE)
                .turnTo(Math.toRadians(175))
                .turnTo(Math.toRadians(185))
                .turnTo(Math.toRadians(175))
                .turnTo(Math.toRadians(185))
                .turnTo(Math.toRadians(175))
                .turnTo(Math.toRadians(185))
                .turnTo(Math.toRadians(175))
                .turnTo(Math.toRadians(185))
                .turnTo(Math.toRadians(175))
                .turnTo(Math.toRadians(185))
                .turnTo(Math.toRadians(175))
                .turnTo(Math.toRadians(185))
                .turnTo(Math.toRadians(175))
                .turnTo(Math.toRadians(185));

        movement1a = robot.driveSubsystem.cancel(movement1.build());

        CommandScheduler.getInstance().schedule(new BucketInitializeCommand(robot));

    }

    @Override
    public void init_loop() {
        robot.clearCache();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new ExtendAndSpinCommand(robot, Set.of(YELLOW, BLUE), Globals.EXTENDO_MAX_EXTENSION).whenFinished(() -> {
                            Log.i("IntakeToHighBucketAuto", "done");
                            movement1a.cancelAbruptly();
                        }),
                                new ActionCommand(movement1a, Collections.emptySet())
                )
        );

    }

    private String detectColor(float hue, float sat, float val) {
        if (val < 0.2) return "Too Dark";

        // RED
        if ((hue >= 0 && hue <= 25) || (hue >= 330 && hue <= 360)) {
            if (sat > 0.4 && val > 0.2) return "Red";
        }

        // BLUE
        if (hue >= 200 && hue <= 250) {
            if (sat > 0.4 && val > 0.2) return "Blue";
        }

        // YELLOW
        if (hue >= 65 && hue <= 100) {
            if (sat > 0.7 && val > 12) return "Yellow";
        }

        return "Unknown";
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.extendoSubsystem.extendoSlidesLoop();
        robot.outtakeSlidesSubsystem.outtakeSlidesLoop();
        float[] hsv = new float[3];
        int r = robot.colorSensor.red();
        int g = robot.colorSensor.green();
        int b = robot.colorSensor.blue();

        // Normalize and convert to HSV
        Color.RGBToHSV(r * 8, g * 8, b * 8, hsv); // Scale to 0–255

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        String colorDetected = detectColor(hue, sat, val);

        telemetry.addData("Detected Color", colorDetected);

        robot.clearCache();

    }

    @Override
    public void stop() {
        telemetry.addLine("Ended OpMode.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}
