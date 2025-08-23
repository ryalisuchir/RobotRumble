package org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.AutoSkib;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.MAX_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.MIN_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.opmode.Testing.ColorSensorHSV.detectSampleColor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.ExtendAndSpinCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

import java.util.Set;

public class JustIntakeEverythingCommand extends CommandBase {

    private boolean resetDone = false;

    private final Robot robot;
    private final double extendoTargetPosition;
    public static boolean colorDetected = false;
    long lastEncoderUpdateTime = 0;

    public JustIntakeEverythingCommand(
            Robot robot,
            double extendoTargetPosition
    ) {
        this.extendoTargetPosition = extendoTargetPosition;
        this.robot = robot;

//        addRequirements(robot.extendoSubsystem, robot.spinnerSubsystem, robot.gateSubsystem); i um don't like solverslib anymore
    }


    public static boolean goofy() {
        return colorDetected;
    }

    @Override
    public void initialize() {
        colorDetected = false;
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(extendoTargetPosition)),
                new ParallelCommandGroup(
                                new ExtendoSlidesCommand(robot.extendoSubsystem, extendoTargetPosition),
                                new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.INTAKE),
                        new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.INTAKING),
                        new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED)
                )
                )
        );
    }

    @Override
    public void execute() {
        if (!resetDone) {
            CommandScheduler.getInstance().schedule(
                    new UninterruptibleCommand(
                            new OuttakeTransferReadyCommand(robot)
                    )
            );
            resetDone = true;
        }

        if (colorDetected) return;
        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        if (robot.intakeSpinner.getCurrent(CurrentUnit.MILLIAMPS) > 6000) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.REVERSED),
                            new WaitCommand(100),
                            new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.INTAKING)
                    )
            );
            lastEncoderUpdateTime = System.currentTimeMillis();
        }


        if (distance > MIN_DISTANCE_THRESHOLD && distance < MAX_DISTANCE_THRESHOLD) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED),
                                        new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER)
                                )
                        )
                );
        }
    }


    @Override
    public boolean isFinished() {
        return colorDetected;
    }

    @Override
    public void end(boolean interrupted) {
        resetDone = false;
    }
}
