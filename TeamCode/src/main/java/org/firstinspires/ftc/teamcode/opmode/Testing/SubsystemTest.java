package org.firstinspires.ftc.teamcode.opmode.Testing;

import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.BLUE;
import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.YELLOW;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.IntakeToHighBucket;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenDeposit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenGrab;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;


import java.util.Set;

@TeleOp
public class SubsystemTest extends CommandOpMode {
    GamepadEx ahnafButtonController;
    public Robot robot;
    public double root = 1;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new Pose2d(0,0,Math.toRadians(0)), true, false);
        robot.extendoSubsystem.extendoSetPosition(0);
        robot.outtakeSlidesSubsystem.outtakeSetPosition(0);
        schedule(new OuttakeTransferReadyCommand(robot));
        ahnafButtonController = new GamepadEx(gamepad1);
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
    public void run() {
        CommandScheduler.getInstance().run();
        robot.extendoSubsystem.extendoSlidesLoop();
        robot.outtakeSlidesSubsystem.outtakeSlidesLoop();
        telemetry.addData("Extendo:", robot.extendoMotor.getCurrentPosition());
        telemetry.addData("Lift:", robot.rightLift.getCurrentPosition());

        robot.clearCache();

        telemetry.addData("Distance:", robot.colorSensor.getDistance(DistanceUnit.CM));
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
        telemetry.update();

        if (gamepad1.ps) {
            schedule(new ExtendAndSpinCommand(robot, Set.of(YELLOW, BLUE), Globals.EXTENDO_MAX_EXTENSION * 1));
        }

        robot.pinPointDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -0.5 * Math.tan(1.12 * gamepad1.left_stick_y),
                        -0.5 * Math.tan(1.12 * gamepad1.left_stick_x)
                ),
                -0.5 * Math.tan(1.12 * gamepad1.right_stick_x)
        ));
        telemetry.addData("Root: ", root);

        if (gamepad1.triangle) {
            schedule(new IntakeToHighBucket(robot, Set.of(YELLOW, BLUE), Globals.EXTENDO_MAX_EXTENSION));
        }

        if (gamepad1.cross) {
            schedule(new SpecimenGrab(robot));
        }

        if (gamepad1.circle) {
                schedule(new SpecimenDeposit(robot));
        }

        if (gamepad1.square) {
            if (Globals.outtakeClawState == Globals.OuttakeClawState.OPEN) {
                schedule(new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED));
            } else {
                schedule(new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN));
            }
        }

        if (gamepad1.dpad_left) {
            schedule (new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN));
        }

        if (gamepad1.dpad_up) {
            schedule(new OuttakeDepositHighCommand(robot));
        }

        if (gamepad1.dpad_down) {
            schedule(new OuttakeTransferReadyCommand(robot));
        }

    }
}
