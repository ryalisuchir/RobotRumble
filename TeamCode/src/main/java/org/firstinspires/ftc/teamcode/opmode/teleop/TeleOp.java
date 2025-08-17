package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.BLUE;
import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.YELLOW;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenGrab;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenDeposit;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;


import java.util.Set;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
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

        //AHNAF LOOK HERE FOR NEW COMMAND:
//        new InstantCommand(() -> {
//            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        })

        robot.pinPointDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -0.5 * Math.tan(1.12 * gamepad1.left_stick_y),
                        -0.5 * Math.tan(1.12 * gamepad1.left_stick_x)
                ),
                -0.5 * Math.tan(1.12 * gamepad1.right_stick_x)
        ));
        telemetry.addData("Root: ", root);

        if (gamepad1.right_trigger >= .5 && root != 2) {
            root = 2;
//            schedule(new ExtendAndSpinCommand(robot, Set.of(YELLOW, BLUE), Globals.EXTENDO_MAX_EXTENSION * 0.86));
            schedule(
                    new ParallelCommandGroup(
                            new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.INTAKE),
                            new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.INTAKING),
                            new GateCommand(robot.gateSubsystem, Globals.gateState.CLOSED)

                    )
            );
        }
        if (gamepad1.right_trigger < .5 && root!=1) {
            root = 1;
            schedule(
                    new SequentialCommandGroup(
                            new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED),
                            new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.READY),
                            new WaitCommand(200)

                    )
            );
        }
        if (gamepad1.left_trigger > 0 && root != 3){
            root = 3;
            schedule(
                    new SequentialCommandGroup(
                            new SpinnerCommand(robot.spinnerSubsystem,Globals.SpinnerState.REVERSED),
                            new DropdownCommand(robot, robot.dropdownSubsystem,Globals.DropdownState.READY)
                    )
            );
        }
        if (gamepad1.left_trigger > .8 && root != 4){
            root = 4;
            schedule(
                    new SequentialCommandGroup(
                            new SpinnerCommand(robot.spinnerSubsystem,Globals.SpinnerState.REVERSED),
                            new DropdownCommand(robot, robot.dropdownSubsystem,Globals.DropdownState.INTAKE)
                    )
            );
        }
        if (gamepad1.left_trigger == 0 && (root ==4 || root == 3)) {
            root = 5;
            schedule(new SequentialCommandGroup(new SpinnerCommand(robot.spinnerSubsystem, Globals.spinnerState.STOPPED)));
        }

        if (gamepad2.square) {
            schedule(
                    new ParallelCommandGroup(
                            new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.READY),
                            new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_EXTENSION*1)
                    )
            );
        }
        if (gamepad2.circle) {
            schedule(
                    new ParallelCommandGroup(
                            new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.READY),
                            new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_EXTENSION*.5)
                    )
            );
        }
        if (gamepad2.cross) {
            schedule(
                    new ParallelCommandGroup(
                            new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.READY),
                            new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION)
                    )
            );
        }
        if (gamepad1.right_bumper) {
            schedule(
                    new TransferCommand(robot)
            );
        }

        if (gamepad1.dpad_right) {
            schedule (
                    new ParallelCommandGroup(
                            new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED),
                            new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER),
                            new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION)
                    )
            );
        }


        if (gamepad2.dpad_up) {
            schedule(new OuttakeDepositHighCommand(robot));
        }

        if (gamepad2.dpad_down) {
            schedule(new OuttakeTransferReadyCommand(robot));
        }

        if (gamepad1.left_bumper) {
            schedule(new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN));
        }


        if(gamepad1.dpad_up) {
            schedule( new SpecimenGrab(robot));
        }
        if(gamepad1.dpad_down) {
            schedule(new SpecimenDeposit(robot));
        }
    }
}
