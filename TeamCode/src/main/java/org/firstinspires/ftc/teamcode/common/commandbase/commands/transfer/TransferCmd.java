package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class TransferCmd extends SequentialCommandGroup {
    public TransferCmd(Robot robot) {
        addCommands(
                new UninterruptibleCommand(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                      new WaitCommand(100),
                                        new GateCommand(robot.gateSubsystem, Globals.GateState.OPEN_TRANSFER)
                                ),
                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED),
                                new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER),
                                new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION)
                        ),
                        new ParallelCommandGroup(
                                new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_POS),
                                new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.COCKED_UP)
                        ),
                        new WaitCommand(100),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
                        new WaitCommand(50)
                )
                )
        );
    }
}
