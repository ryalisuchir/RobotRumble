package org.firstinspires.ftc.teamcode.common.commandbase.commands.intake;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class RaiseStopCmd extends ParallelCommandGroup {
    public RaiseStopCmd(Robot robot) {
        addCommands(
                new ParallelCommandGroup(
                        new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER),
                        new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED)
                )
        );
    }
}
