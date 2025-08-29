package org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.teleop;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class EjectCmd extends SequentialCommandGroup {
    public EjectCmd(Robot robot) {
        addCommands(
                new SequentialCommandGroup(
                        new GateCommand(robot.gateSubsystem, Globals.GateState.OPEN_EJECT),
                        new ParallelCommandGroup(
                            new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.INTAKE),
                            new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.INTAKING)
                        )
                )
        );
    }
}
