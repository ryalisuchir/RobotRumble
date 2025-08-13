package org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class OuttakeTransferReadyCommand extends SequentialCommandGroup {
    public OuttakeTransferReadyCommand(Robot robot) {
        addCommands(
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new SequentialCommandGroup( //true = just come down
                                    new ParallelCommandGroup(
                                            new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_READY_POS),
                                            new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER),
                                            new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.TRANSFER),
                                            new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN_TRANSFER)
                                    ),
                                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_READY_POS)
                                ),
                                new SequentialCommandGroup( //false = regular transfer (SAME THING JUST FOR NOW)
                                                new ParallelCommandGroup(
                                                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_READY_POS),
                                                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER),
                                                        new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.TRANSFER),
                                                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN_TRANSFER)
                                                )
                                ),
                                () -> robot.rightLift.getCurrentPosition() >= Globals.LIFT_READY_DEPOSIT_POS
                        )
                )
        );
    }
}
