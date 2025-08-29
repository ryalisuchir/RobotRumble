package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.EXTENDO_TRANSFER_FACTOR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class TransferReadyCmd extends SequentialCommandGroup {
    public TransferReadyCmd(Robot robot) {
        addCommands(
                new ParallelCommandGroup(
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER),
                        new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.TRANSFER),
                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_READY_POS),
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN_TRANSFER)
                )
        );
    }
}
