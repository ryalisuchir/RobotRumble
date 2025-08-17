package org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

import java.util.Set;

public class IntakeToHighBucket extends SequentialCommandGroup {
    public IntakeToHighBucket(Robot robot, Set<ExtendAndSpinCommand.SampleColorDetected> targetColors, double extendoPosition) {
        addCommands(
                new ExtendAndSpinCommand(robot, targetColors, extendoPosition),
                new TransferCommand(robot)
        );
    }
}
