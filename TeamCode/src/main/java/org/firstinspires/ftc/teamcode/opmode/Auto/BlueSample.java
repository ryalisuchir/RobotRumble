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
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.AutoSkib.DrivingResetExtendoOut;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.AutoSkib.ExtendAndSpinAndStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.AutoSkib.JustIntakeEverythingCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.AutoSkib.RetractTransferLiftAndExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.IntakeToHighBucket;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.TransferToDepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.RetractNResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Collections;
import java.util.Set;

@Autonomous
public class BlueSample extends OpMode {
    Robot robot;
    TrajectoryActionBuilder movement1, movement2, movement3, movement4, movement5, movement6, movement7, movement8, movement9, movement10, movement11, movement12, movement13;
    Action movement1a, movement2a, movement3a, movement4a, movement5a, movement6a, movement8a, movement9a, movement11a, movement12a, movement13a;

    MecanumDrive.CancelableAction movement7a, movement10a;

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
                .turnTo(Math.toRadians(220))
                .turnTo(Math.toRadians(230))
                .turnTo(Math.toRadians(220))
                .turnTo(Math.toRadians(230))
                .turnTo(Math.toRadians(220))
                .turnTo(Math.toRadians(230))
                .turnTo(Math.toRadians(215))
                .turnTo(Math.toRadians(235))
                .turnTo(Math.toRadians(215))
                .turnTo(Math.toRadians(235))
                .turnTo(Math.toRadians(215))
                .turnTo(Math.toRadians(235));

        movement8 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(57.37, 52.32), Math.toRadians(45));

        movement9 = movement8.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(26.21, 10.53), Math.toRadians(225.00)); //extendo will be out at this point. slides should come down, reset.

        movement10 = movement9.endTrajectory().fresh()
                .turnTo(Math.toRadians(220))
                .turnTo(Math.toRadians(230))
                .turnTo(Math.toRadians(220))
                .turnTo(Math.toRadians(230))
                .turnTo(Math.toRadians(220))
                .turnTo(Math.toRadians(230))
                .turnTo(Math.toRadians(215))
                .turnTo(Math.toRadians(235))
                .turnTo(Math.toRadians(215))
                .turnTo(Math.toRadians(235))
                .turnTo(Math.toRadians(215))
                .turnTo(Math.toRadians(235));

        movement11 = movement10.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(57.37, 52.32), Math.toRadians(45));

        movement1a = movement1.build();
        movement2a = movement2.build();
        movement3a = movement3.build();
        movement4a = movement4.build();
        movement5a = movement5.build();
        movement6a = movement6.build();
        movement7a = robot.driveSubsystem.cancel(movement7.build());
        movement8a = movement8.build();
        movement9a = movement9.build();
        movement10a = robot.driveSubsystem.cancel(movement10.build());
        movement11a = movement11.build();

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
                                        new WaitCommand(500),
                                        new OuttakeDepositHighCommand(robot)
                                )
                        ),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new RetractNResetCommand(robot),
                                new UninterruptibleCommand(new JustIntakeEverythingCommand(robot, Globals.EXTENDO_MAX_EXTENSION*1.5).whenFinished(() -> Log.i("IntakeToHighBucketAuto", "done")))
                        ),
                        new ParallelCommandGroup(
                                new ActionCommand(movement2a, Collections.emptySet()),
                                        new SequentialCommandGroup(
                                                new TransferCommand(robot),
                                                new ParallelCommandGroup(
                                                        new OuttakeDepositHighCommand(robot),
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(500), //wait till slides go up on second samp before slides come out
                                                                new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_EXTENSION)
                                                        )
                                                )
                                        )
                        ),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new RetractNResetCommand(robot),
                                new UninterruptibleCommand(new JustIntakeEverythingCommand(robot, Globals.EXTENDO_MAX_EXTENSION*1.5).whenFinished(() -> Log.i("IntakeToHighBucketAuto", "done")))
                        ),
                        new ParallelCommandGroup(
                                new ActionCommand(movement3a, Collections.emptySet()),
                                new SequentialCommandGroup(
                                        new TransferCommand(robot),
                                        new ParallelCommandGroup(
                                                new OuttakeDepositHighCommand(robot),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(500), //wait till slides go up on second samp before slides come out
                                                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_EXTENSION*0.4)
                                                )
                                        )
                                )
                        ),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new ActionCommand(movement4a, Collections.emptySet()),
                                        new UninterruptibleCommand(new JustIntakeEverythingCommand(robot, Globals.EXTENDO_MAX_EXTENSION*1.5).whenFinished(() -> Log.i("IntakeToHighBucketAuto", "done")))
                                ),
                                new RetractNResetCommand(robot)
                        ),
                        new ParallelCommandGroup(
                                new ActionCommand(movement5a, Collections.emptySet()),
                                new SequentialCommandGroup(
                                        new TransferCommand(robot),
                                        new ParallelCommandGroup(
                                                new OuttakeDepositHighCommand(robot),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(500), //wait till slides go up on second samp before slides come out
                                                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_EXTENSION*0.8)
                                                )
                                        )
                                )
                        )
//                        new ParallelCommandGroup( //driving to the sub for 1
//                                new DrivingResetExtendoOut(robot),
//                                new ActionCommand(movement6a, Collections.emptySet())
//                        ),
//                        new ParallelCommandGroup(
//                                new ExtendAndSpinCommand(robot, Set.of(YELLOW, BLUE), Globals.EXTENDO_MAX_EXTENSION).whenFinished(() -> {
//                                    Log.i("IntakeToHighBucketAuto", "done");
//                                    movement7a.cancelAbruptly();
//                                }),
//                                new ActionCommand(movement7a, Collections.emptySet())
//                        ),
//                        new ParallelCommandGroup(
//                                new RetractTransferLiftAndExtendCommand(robot),
//                                new ActionCommand(movement8a, Collections.emptySet())
//                        ),
//                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
//                        new WaitCommand(100),
//                        new ParallelCommandGroup( //driving to the sub for 2!!
//                                new DrivingResetExtendoOut(robot),
//                                new ActionCommand(movement9a, Collections.emptySet())
//                        ),
//                        new ParallelCommandGroup(
//                                new ExtendAndSpinCommand(robot, Set.of(YELLOW, BLUE), Globals.EXTENDO_MAX_EXTENSION).whenFinished(() -> {
//                                    Log.i("IntakeToHighBucketAuto", "done");
//                                    movement10a.cancelAbruptly();
//                                }),
//                                new ActionCommand(movement10a, Collections.emptySet())
//                        ),
//                        new ParallelCommandGroup(
//                                new RetractTransferLiftAndExtendCommand(robot),
//                                new ActionCommand(movement11a, Collections.emptySet())
//                        ),
//                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
//                        new WaitCommand(100)
                )
        );

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.extendoSubsystem.extendoSlidesLoop();
        robot.outtakeSlidesSubsystem.outtakeSlidesLoop();

        robot.clearCache();

    }

    @Override
    public void stop() {
        telemetry.addLine("Ended OpMode.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}
