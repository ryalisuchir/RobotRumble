package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.BLUE;
import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.YELLOW;

import android.graphics.Color;
import android.util.Log;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.BucketInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenDeposit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenGrab;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

import java.util.Collections;
import java.util.Set;

@Autonomous
public class Blue5Spec extends OpMode {
    Robot robot;
    TrajectoryActionBuilder movement1, movement2, movement3, movement4, movement5, movement6, movement7, movement8;
    Action movement1a, movement2a, movement3a, movement4a, movement5a, movement6a, movement7a, movement8a;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, Globals.BLUE_FAR_START_POSE, true, true);

        telemetry.addLine("Initialized.");
        telemetry.update();

        movement1 = robot.driveSubsystem.trajectoryActionBuilderCorrection(Globals.BLUE_FAR_START_POSE)
                .splineToLinearHeading(new Pose2d(-7, 34, Math.toRadians(-90)), Math.toRadians(-90));

        movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-35.43, 33.17), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-48, 10.53), Math.toRadians(100.00));

        movement3 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(-45, 10.53, Math.toRadians(-90))) // stop here
                .strafeToConstantHeading(new Vector2d(-48, 50))
                .strafeToConstantHeading(new Vector2d(-48, 10.53))
                .strafeToConstantHeading(new Vector2d(-64, 10.53))
                .strafeToConstantHeading(new Vector2d(-64, 50))
                .strafeToConstantHeading(new Vector2d(-64, 10.53))
                .strafeToConstantHeading(new Vector2d(-69, 10.53));

        movement4 = robot.driveSubsystem.trajectoryActionBuilderCorrection(new Pose2d(-65, 10.53, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(-69, 50))
                .strafeToConstantHeading(new Vector2d(-60, 60),
                        new TranslationalVelConstraint(25))
        ;

        movement5 = robot.driveSubsystem.trajectoryActionBuilderCorrection(new Pose2d(-65, 54, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(-5.48, 30.91), Math.toRadians(-90));

        movement6 = movement5.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-29, 55, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(
                        new Pose2d(-29, 64, Math.toRadians(90)), Math.toRadians(90),
                        new TranslationalVelConstraint(10)
                );

        movement7 = movement6.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 38, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-10, 32, Math.toRadians(-90)), Math.toRadians(-90));

        movement8 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(-17, 58, Math.toRadians(0)), Math.toRadians(180),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                );

        movement1a = movement1.build();
        movement2a = movement2.build();
        movement3a = movement3.build();
        movement4a = movement4.build();
        movement5a = movement5.build();
        movement6a = movement6.build();
        movement7a = movement7.build();

        CommandScheduler.getInstance().schedule(new SpecimenInitializeCommand(robot));

    }

    @Override
    public void init_loop() {
        robot.clearCache();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ActionCommand(movement1a, Collections.emptySet()),
                                new SpecimenDeposit(robot)
                        ),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
                        new ParallelCommandGroup(
                                new ActionCommand(movement2a, Collections.emptySet()),
                                new SpecimenGrab(robot)
                        ),
                        new ActionCommand(movement3a, Collections.emptySet()),
                        new ActionCommand(movement4a, Collections.emptySet()),
                        new ParallelCommandGroup(
                                new SpecimenDeposit(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new ActionCommand(movement5a, Collections.emptySet())
                                )
                        )
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
