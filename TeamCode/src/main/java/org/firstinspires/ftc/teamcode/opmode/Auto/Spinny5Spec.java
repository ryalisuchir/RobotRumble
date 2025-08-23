//package org.firstinspires.ftc.teamcode.opmode.Auto;
//
//import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.BLUE;
//import static org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand.SampleColorDetected.YELLOW;
//
//import android.graphics.Color;
//import android.util.Log;
//
//import com.acmerobotics.roadrunner.AccelConstraint;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.AngularVelConstraint;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.seattlesolvers.solverslib.command.CommandScheduler;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.BucketInitializeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.SpecSpitter;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositHighCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenDeposit;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenGrab;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenInitializeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.TransferCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.ExtendoSlidesCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.SpinnerCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
//import org.firstinspires.ftc.teamcode.common.robot.Globals;
//import org.firstinspires.ftc.teamcode.common.robot.Robot;
//
//import java.util.Collections;
//import java.util.Set;
//
//@Autonomous
//public class Spinny5Spec extends OpMode {
//    Robot robot;
//    TrajectoryActionBuilder movement1, movement2, movement3, movement4, movement5, movement6, movement7, movement8, movement9, movement10, movement11, movement12, movement13, movement14, movement15;
//    Action movement1a, movement2a, movement3a, movement4a, movement5a, movement6a, movement7a, movement8a, movement9a, movement10a, movement11a, movement12a, movement13a, movement14a, movement15a;
//        Double distance;
//    @Override
//    public void init() {
//        CommandScheduler.getInstance().reset();
//        robot = new Robot(hardwareMap, Globals.BLUE_FAR_START_POSE, true, true);
//
//        telemetry.addLine("Initialized.");
//        telemetry.update();
//
//        movement1 = robot.driveSubsystem.trajectoryActionBuilderCorrection(Globals.BLUE_FAR_START_POSE)
//                .splineToLinearHeading(new Pose2d(-7, 34, Math.toRadians(-90)), Math.toRadians(-90));
//
//        movement2 = movement1.endTrajectory().fresh()
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-34, 38, Math.toRadians(-134)), Math.toRadians(-134));
//
//        movement3 = movement2.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(-48, 50, Math.toRadians(-223)), Math.toRadians(-223));
//
//        movement4 = movement3.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(-51, 37, Math.toRadians(-134)), Math.toRadians(-134));
//
//        movement5 = movement4.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(-48, 50, Math.toRadians(-223)), Math.toRadians(-223));
//
//        movement6 = movement5.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(-58, 23, Math.toRadians(-180)), Math.toRadians(-180));
//
//        movement7 = movement6.endTrajectory().fresh()
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-48, 50, Math.toRadians(-223)), Math.toRadians(-223));
//
//        movement8 = movement7.endTrajectory().fresh() //grab1
//                .turnTo(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-43, 63, Math.toRadians(-90)), Math.toRadians(-90));
//
//        movement9 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(-43, 63, Math.toRadians(-90)))
//                .splineToLinearHeading(
//                        new Pose2d(-7, 31, Math.toRadians(-90)), Math.toRadians(-90),
//                        null,
//                        new ProfileAccelConstraint(-20, 60)
//                );
//
//        movement10 = movement9.endTrajectory().fresh() //grab2
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-41, 58), Math.toRadians(90))
//                .splineToConstantHeading(
//                        new Vector2d(-41, 63), Math.toRadians(90),
//                        new TranslationalVelConstraint(15)
//                );
//
//        movement11 = movement10.endTrajectory().fresh() //deposit2
//                .setReversed(false)
//                .splineToLinearHeading(
//                        new Pose2d(-7, 32, Math.toRadians(-90)), Math.toRadians(-90),
//                        null,
//                        new ProfileAccelConstraint(-20, 60)
//                );
//
//        movement12 = movement11.endTrajectory().fresh() //grab2
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-41, 58), Math.toRadians(90))
//                .splineToConstantHeading(
//                        new Vector2d(-41, 63), Math.toRadians(90),
//                        new TranslationalVelConstraint(15)
//                );
//
//        movement13 = movement12.endTrajectory().fresh() //deposit2
//                .setReversed(false)
//                .splineToLinearHeading(
//                        new Pose2d(-7, 32, Math.toRadians(-90)), Math.toRadians(-90),
//                        null,
//                        new ProfileAccelConstraint(-20, 60)
//                );
//
//        movement14 = movement13.endTrajectory().fresh() //grab2
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-41, 58), Math.toRadians(90))
//                .splineToConstantHeading(
//                        new Vector2d(-41, 63), Math.toRadians(90),
//                        new TranslationalVelConstraint(15)
//                );
//
//        movement15 = movement14.endTrajectory().fresh() //deposit2
//                .setReversed(false)
//                .splineToLinearHeading(
//                        new Pose2d(-7, 32, Math.toRadians(-90)), Math.toRadians(-90),
//                        null,
//                        new ProfileAccelConstraint(-20, 60)
//                );
//
//        movement1a = movement1.build();
//        movement2a = movement2.build();
//        movement3a = movement3.build();
//        movement4a = movement4.build();
//        movement5a = movement5.build();
//        movement6a = movement6.build();
//        movement7a = movement7.build();
//        movement8a = movement8.build();
//        movement9a = movement9.build();
//        movement10a = movement10.build();
//        movement11a = movement11.build();
//        movement12a = movement12.build();
//        movement13a = movement13.build();
//        movement14a = movement14.build();
//        movement15a = movement15.build();
//
//        CommandScheduler.getInstance().schedule(new SpecimenInitializeCommand(robot));
//
//    }
//
//    @Override
//    public void init_loop() {
//        robot.clearCache();
//        CommandScheduler.getInstance().run();
//    }
//
//    @Override
//    public void start() {
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new ActionCommand(movement1a, Collections.emptySet()),
//                                new SpecimenDeposit(robot)
//                        ),
//                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
//                        new ParallelCommandGroup(
//                                new ActionCommand(movement2a, Collections.emptySet()),
//                                new SpecimenGrab(robot),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(1800),
//                                    new SpecSpitter(robot, Globals.EXTENDO_MAX_EXTENSION*0.8)
//                                )
//                        ),
//                        new ActionCommand(movement3a, Collections.emptySet()),
//                        new SequentialCommandGroup(
//                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.REVERSED),
//                        new WaitCommand(500),
//                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED)
//                        ),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new WaitCommand(700),
//                                        new SpecSpitter(robot, Globals.EXTENDO_MAX_EXTENSION*0.6)
//                                ),
//                                new ActionCommand(movement4a, Collections.emptySet())
//                        ),
//                        new ActionCommand(movement5a, Collections.emptySet()),
//                        new SequentialCommandGroup(
//                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.REVERSED),
//                                new WaitCommand(500),
//                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED)
//                        ),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new WaitCommand(1000),
//                                        new SpecSpitter(robot, Globals.EXTENDO_MAX_EXTENSION*0.6)
//                                ),
//                                new ActionCommand(movement6a, Collections.emptySet())
//                        ),
//                        new ActionCommand(movement7a, Collections.emptySet()),
//                        new SequentialCommandGroup(
//                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.REVERSED),
//                                new WaitCommand(500),
//                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED)
//                        ),
//                        new ParallelCommandGroup(
//                                new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION),
//                                new ActionCommand(movement8a, Collections.emptySet())
//                        ),
//                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
//                        new WaitCommand(100),
//                        new ParallelCommandGroup(
//                                new SpecimenDeposit(robot),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new ActionCommand(movement9a, Collections.emptySet())
//                                )
//                        ),
//                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
//                        new ParallelCommandGroup(
//                                new SpecimenGrab(robot),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new ActionCommand(movement10a, Collections.emptySet())
//                                )
//                        ),
//                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
//                        new WaitCommand(100),
//                        new ParallelCommandGroup(
//                                new SpecimenDeposit(robot),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new ActionCommand(movement11a, Collections.emptySet())
//                                )
//                        ),
//                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
//                        new ParallelCommandGroup(
//                                new SpecimenGrab(robot),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new ActionCommand(movement12a, Collections.emptySet())
//                                )
//                        ),
//                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
//                        new WaitCommand(100),
//                        new ParallelCommandGroup(
//                                new SpecimenDeposit(robot),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new ActionCommand(movement13a, Collections.emptySet())
//                                )
//                        ),
//                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN)
////                        new ParallelCommandGroup(
////                                new SpecimenGrab(robot),
////                                new SequentialCommandGroup(
////                                        new WaitCommand(500),
////                                        new ActionCommand(movement14a, Collections.emptySet())
////                                )
////                        ),
////                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
////                        new WaitCommand(100),
////                        new ParallelCommandGroup(
////                                new SpecimenDeposit(robot),
////                                new SequentialCommandGroup(
////                                        new WaitCommand(500),
////                                        new ActionCommand(movement15a, Collections.emptySet())
////                                )
////                        ),
////                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN)
//                )
//        );
//
//    }
//
//    @Override
//    public void loop() {
//        CommandScheduler.getInstance().run();
//        robot.driveSubsystem.updatePoseEstimate();
//        robot.extendoSubsystem.extendoSlidesLoop();
//        robot.outtakeSlidesSubsystem.outtakeSlidesLoop();
//        distance = robot.colorSensor.getDistance(DistanceUnit.CM);
//
//        telemetry.addData("Dist: ", distance);
//
//        telemetry.addData("Current: ", robot.intakeSpinner.getCurrent(CurrentUnit.MILLIAMPS));
//
//        robot.clearCache();
//
//    }
//
//    @Override
//    public void stop() {
//        telemetry.addLine("Ended OpMode.");
//        telemetry.update();
//        CommandScheduler.getInstance().reset();
//    }
//}
