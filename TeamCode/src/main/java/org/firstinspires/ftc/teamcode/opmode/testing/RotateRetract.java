package org.firstinspires.ftc.teamcode.opmode.testing;
import android.util.Log;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ExtendSpinCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.TransferCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.TransferReadyCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.SampRestCmd;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Collections;
import java.util.Set;

@Autonomous
public class RotateRetract extends OpMode {
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

        CommandScheduler.getInstance().schedule(new SampRestCmd(robot));

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
                        new ExtendSpinCmd(robot, Set.of(Globals.SampleColorDetected.YELLOW, Globals.SampleColorDetected.BLUE), Globals.EXTENDO_MAX_EXTENSION).whenFinished(() -> {
                            Log.i("IntakeToHighBucketAuto", "done");
                            movement1a.cancelAbruptly();
                        }),
                        new ActionCommand(movement1a, Collections.emptySet()),
                        new TransferReadyCmd(robot)
                ),
                        new TransferCmd(robot)
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
