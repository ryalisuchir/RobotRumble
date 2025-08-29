package org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class BucketCmd extends SequentialCommandGroup {
    public BucketCmd(Robot robot, double pos) {
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.UP),
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.BUCKET_IDEAL),
                                new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER)
                        ),
                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, pos),
                        new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.BUCKET_IDEAL)

                )
        );
    }
}
