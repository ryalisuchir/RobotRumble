package org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class OuttakeDepositHighCommand extends SequentialCommandGroup {
    public OuttakeDepositHighCommand(Robot robot) {
        addCommands(
                new ParallelCommandGroup(
                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_HIGH_POS),
                                new SequentialCommandGroup(
                                     new WaitCommand(500),
                                     new ParallelCommandGroup(
                                             new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.BUCKET_IDEAL),
                                             new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.BUCKET_IDEAL)
                                     )
                                )

                )
        );
    }
}
