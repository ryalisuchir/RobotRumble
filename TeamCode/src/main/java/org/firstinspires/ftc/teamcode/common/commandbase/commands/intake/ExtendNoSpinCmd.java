package org.firstinspires.ftc.teamcode.common.commandbase.commands.intake;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class ExtendNoSpinCmd extends SequentialCommandGroup {
    public ExtendNoSpinCmd(Robot robot, double pos) {
        addCommands(
                new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ExtendoSlidesCommand(robot.extendoSubsystem, pos),
                                        new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.READY),
                                        new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED)
                                )
                )
        );
    }
}