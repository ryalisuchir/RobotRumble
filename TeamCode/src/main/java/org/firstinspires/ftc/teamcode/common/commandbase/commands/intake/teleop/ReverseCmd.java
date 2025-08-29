package org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.teleop;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class ReverseCmd extends SequentialCommandGroup {
    public ReverseCmd(Robot robot) {
        addCommands(
                        new ParallelCommandGroup(
                                new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.READY),
                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.REVERSED)
                        )
        );
    }
}
