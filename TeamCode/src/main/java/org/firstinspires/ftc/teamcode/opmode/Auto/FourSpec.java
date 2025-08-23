package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenDeposit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenGrab;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SpecimenInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

import java.util.Collections;

@Autonomous
public class FourSpec extends OpMode {
    Robot robot;
    TrajectoryActionBuilder movement1, movement2, movement3, movement4, movement5, movement6, movement7, movement8, movement9, movement10;
    Action movement1a, movement2a, movement3a, movement4a, movement5a, movement6a, movement7a, movement8a, movement9a, movement10a;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, Globals.BLUE_FAR_START_POSE, true, true);

        movement1 = robot.driveSubsystem.trajectoryActionBuilderCorrection(Globals.BLUE_FAR_START_POSE)
                .splineToLinearHeading(new Pose2d(-7, 34, Math.toRadians(-90)), Math.toRadians(-90));

        movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-35.43, 33.17), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-48, 17), Math.toRadians(100.00));

        movement3 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(-48, 17, Math.toRadians(-90))) // stop here
                .strafeToConstantHeading(
                        new Vector2d(-48, 50),
                        null,
                        new ProfileAccelConstraint(-80, 80)
                )
                .strafeToConstantHeading(
                        new Vector2d(-48, 13),
                        null,
                        new ProfileAccelConstraint(-80, 80)
                )
                .strafeToConstantHeading(
                        new Vector2d(-57, 13),
                        null,
                        new ProfileAccelConstraint(-80, 80)
                )
                .strafeToConstantHeading(
                        new Vector2d(-57, 50),
                        null,
                        new ProfileAccelConstraint(-80, 80)
                );

        movement4 = robot.driveSubsystem.trajectoryActionBuilderCorrection(new Pose2d(-57, 50, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(-60, 64),
                        new TranslationalVelConstraint(15))
        ;

        movement5 = movement4.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(
                        new Vector2d(-7, 32), Math.toRadians(-90),
                        null,
                        new ProfileAccelConstraint(-40, 60)
                );

        movement6 = movement5.endTrajectory().fresh() //grab2
                .setReversed(true)
                .splineTo(
                        new Vector2d(-41, 65), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-15, 60)
                );

        movement7 = movement6.endTrajectory().fresh() //deposit2
                .setReversed(false)
                .splineToConstantHeading(
                        new Vector2d(-7, 32), Math.toRadians(-90),
                        null,
                        new ProfileAccelConstraint(-40, 60)
                );

        movement8 = movement7.endTrajectory().fresh() //grab2
                .setReversed(true)
                .splineTo(
                        new Vector2d(-41, 65.5), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-15, 60)
                );

        movement9 = movement8.endTrajectory().fresh() //deposit2
                .setReversed(false)
                .splineToConstantHeading(
                        new Vector2d(-7, 32), Math.toRadians(-90),
                        null,
                        new ProfileAccelConstraint(-40, 60)
                );

        movement10 = movement9.endTrajectory().fresh() //deposit2
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(-56, 50, Math.toRadians(-180)), Math.toRadians(-180)
                );

        movement1a = movement1.build();
        telemetry.addLine("1.");
        movement2a = movement2.build();
        telemetry.addLine("2.");
        movement3a = movement3.build();
        telemetry.addLine("3.");
        movement4a = movement4.build();
        telemetry.addLine("4.");
        movement5a = movement5.build();
        telemetry.addLine("5.");
        movement6a = movement6.build();
        telemetry.addLine("6.");
        movement7a = movement7.build();
        telemetry.addLine("7.");
        movement8a = movement8.build();
        telemetry.addLine("8.");
        movement9a = movement9.build();
        telemetry.addLine("9.");
        movement10a = movement10.build();

        CommandScheduler.getInstance().schedule(new SpecimenInitializeCommand(robot));
        telemetry.addLine("Done.");
        telemetry.update();

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
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SpecimenGrab(robot)
                                )
                        ),
                        new ActionCommand(movement3a, Collections.emptySet()),
                        new ActionCommand(movement4a, Collections.emptySet()),
                        new WaitCommand(200),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SpecimenDeposit(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ActionCommand(movement5a, Collections.emptySet())
                                )
                        ),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SpecimenGrab(robot)
                                ),
                                        new ActionCommand(movement6a, Collections.emptySet())
                        ),
                        new WaitCommand(200),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SpecimenDeposit(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new ActionCommand(movement7a, Collections.emptySet())
                                        )
                        ),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SpecimenGrab(robot)
                                ),
                                        new ActionCommand(movement8a, Collections.emptySet())
                        ),
                        new WaitCommand(200),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SpecimenDeposit(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ActionCommand(movement9a, Collections.emptySet())
                                )
                        ),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SpecimenGrab(robot)
                                ),
                                new ActionCommand(movement10a, Collections.emptySet())
                        )
                ));

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
