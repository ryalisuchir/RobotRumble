package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.BLUE;
import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.RED;
import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.YELLOW;

import android.graphics.Color;
import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.BucketInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.IntakeToHighBucket;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.TransferToDepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

import java.util.Collections;
import java.util.Set;

@Autonomous
public class BlueSample extends OpMode {
    Robot robot;
    TrajectoryActionBuilder movement1, movement2, movement3, movement4, movement5, movement6, movement7;
    Action movement1a, movement2a, movement3a, movement4a, movement5a, movement6a, movement7a, movement8a, movement9a;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, Globals.BLUE_SIDEWAYS_START_POSE, true, true);

        telemetry.addLine("Initialized.");
        telemetry.update();

        movement1 = robot.driveSubsystem.trajectoryActionBuilderCorrection(Globals.BLUE_SIDEWAYS_START_POSE)
                .splineToLinearHeading(
                        new Pose2d(59, 57, Math.toRadians(225)), Math.toRadians(225),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                );

        movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(
                        new Pose2d(51, 60, Math.toRadians(270)), (Math.toRadians(0)),
                        null,
                        new ProfileAccelConstraint(-35, 35)
                );

        movement3 = movement2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(61, 57, Math.toRadians(225)), Math.toRadians(225));

        movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(
                                48, 48, Math.toRadians(270)), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-35, 35)
                );

        movement5 = movement4.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(64, 57, Math.toRadians(225)), Math.toRadians(225));

        movement6 = movement5.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(53, 47, Math.toRadians(-53)), Math.toRadians(40),
                        new AngularVelConstraint(Math.PI * 0.8)
                );

        movement7 = movement6.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(61, 56.5, Math.toRadians(225)), Math.toRadians(225));

        movement1a = movement1.build();
        movement2a = movement2.build();
        movement3a = movement3.build();
        movement4a = movement4.build();
        movement5a = movement5.build();
        movement6a = movement6.build();
        movement7a = movement7.build();

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
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ActionCommand(movement1a, Collections.emptySet()),
                                new OuttakeDepositHighCommand(robot)
                        ),
                        new WaitCommand(100),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
                        new ParallelCommandGroup(
                                new ActionCommand(movement2a, Collections.emptySet()),
                                new OuttakeTransferReadyCommand(robot),
                                new SequentialCommandGroup(
                                 new WaitCommand(1000),
                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, 0),
                                                        new InstantCommand(() -> {
                                                            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                                            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                                            robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                                            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                                        })
                                                ),
                                                new UninterruptibleCommand(new ExtendAndSpinCommand(robot, Set.of(YELLOW, BLUE), Globals.EXTENDO_MAX_EXTENSION).whenFinished(() -> Log.i("IntakeToHighBucketAuto", "done")))
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                new ActionCommand(movement3a, Collections.emptySet()),
                                        new SequentialCommandGroup(
                                                new TransferCommand(robot),
                                                new WaitCommand(100),
                                                new OuttakeDepositHighCommand(robot)
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
