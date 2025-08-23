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
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.AutoSkib.ExtendAndSpinAndStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.AutoSkib.JustIntakeEverythingCommand;
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
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(57, 50, Math.toRadians(250)), Math.toRadians(0),
                        null,
                        new ProfileAccelConstraint(-30, 85)
                ); //dep 1, grab second

        movement2 = movement1.endTrajectory().fresh()
                .splineToLinearHeading(
                        new Pose2d(57, 50, Math.toRadians(270)), Math.toRadians(270)
                ); //transfer from samp2, outtake slides go up, extendo goes out to grab other one. when outtake slides are done, they will come down, reset, extendo will go even more out

        movement3 = movement2.endTrajectory().fresh()
                .splineToLinearHeading(
                        new Pose2d(57, 50, Math.toRadians(250)), Math.toRadians(0),
                        null,
                        new ProfileAccelConstraint(-30, 85)
                ); //deposit 2, extendo comes out a little

        movement4 = movement3.endTrajectory().fresh()
                .splineToLinearHeading(
                        new Pose2d(53, 45, Math.toRadians(315)), Math.toRadians(315)
                ); //turn, extend how much ever, spin

        movement5 = movement4.endTrajectory().fresh()
                .splineToLinearHeading(
                        new Pose2d(57, 50, Math.toRadians(250)), Math.toRadians(0),
                        null,
                        new ProfileAccelConstraint(-30, 85)
                ); //transfer while doing this mvoement, go up. extendo should come out here like 0.4*max extension

        movement6 = movement5.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(26.21, 10.53), Math.toRadians(225.00)); //extendo will be out at this point. slides should come down, reset.

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
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new OuttakeDepositHighCommand(robot)
                                )
                        ),
                        new WaitCommand(100),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new OuttakeTransferReadyCommand(robot),
                                        new UninterruptibleCommand(
                                                new SequentialCommandGroup(
                                                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_RETRACT_POS),
                                                        new WaitCommand(100),
                                                        new InstantCommand(() -> {
                                                            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                                            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                                            robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                                            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                                        })
                                                )
                                        )
                                ),
                                new JustIntakeEverythingCommand(robot, Globals.EXTENDO_MAX_EXTENSION*1.5).whenFinished(() -> Log.i("IntakeToHighBucketAuto", "done"))
                        ),
                        new ParallelCommandGroup(
                                new ActionCommand(movement2a, Collections.emptySet()),
                                        new SequentialCommandGroup(
                                                new TransferCommand(robot),
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
