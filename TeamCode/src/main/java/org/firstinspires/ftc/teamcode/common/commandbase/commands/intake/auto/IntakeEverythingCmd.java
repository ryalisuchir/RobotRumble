package org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.auto;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.MAX_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.MIN_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.maxSpinnerCurrent;

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
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.RaiseStopCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

import java.util.Set;

public class IntakeEverythingCmd extends CommandBase {
    private final Robot robot;
    private final double extendoTargetPosition;
    public static boolean colorDetected = false;
    long lastEncoderUpdateTime = 0;

    public IntakeEverythingCmd(
            Robot robot,
            double extendoTargetPosition
    ) {
        this.extendoTargetPosition = extendoTargetPosition;
        this.robot = robot;
    }

    boolean commandCompleted = false;

    public static boolean goofy() {
        return colorDetected;
    }

    @Override
    public void initialize() {
        colorDetected = false;
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new ExtendoSlidesCommand(robot.extendoSubsystem, extendoTargetPosition),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                        new ParallelCommandGroup(
                                new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.INTAKE),
                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.INTAKING),
                                new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED)
                        )
                        )
                )
        );
    }

    @Override
    public void execute() {

        if (colorDetected) return;
        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        if (robot.intakeSpinner.getCurrent(CurrentUnit.MILLIAMPS) >= maxSpinnerCurrent) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.REVERSED),
                            new WaitCommand(100),
                            new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED),
                            new InstantCommand(() -> commandCompleted = true)
                    )
            );
        }

        if (distance > MIN_DISTANCE_THRESHOLD && distance < MAX_DISTANCE_THRESHOLD && robot.intakeSpinner.getCurrent(CurrentUnit.MILLIAMPS) < maxSpinnerCurrent) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new RaiseStopCmd(robot)
                    ),
                    new InstantCommand(() -> commandCompleted = true)
            );
        }
    }


    @Override
    public boolean isFinished() {
        return commandCompleted;
    }

    @Override
    public void end(boolean interrupted) {
        commandCompleted = false;
    }
}
