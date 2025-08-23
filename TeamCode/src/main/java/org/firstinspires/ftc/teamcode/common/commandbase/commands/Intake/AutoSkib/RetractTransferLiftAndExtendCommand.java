package org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.AutoSkib;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class RetractTransferLiftAndExtendCommand extends SequentialCommandGroup {
    public RetractTransferLiftAndExtendCommand(Robot robot) {
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeTransferReadyCommand(robot),
                                new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED),
                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED),
                                new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER),
                                new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION)
                        ),
                        new ParallelCommandGroup(
                                new GateCommand(robot.gateSubsystem, Globals.GateState.OPEN_TRANSFER),
                                new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_POS)
                        ),
                        new WaitCommand(100),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
                        new ParallelCommandGroup(
                                new OuttakeDepositHighCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION*0.8)
                                )
                        )
                )
        );
    }
}
