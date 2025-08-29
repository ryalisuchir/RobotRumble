package org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class BucketCmd extends SequentialCommandGroup {
    public BucketCmd(Robot robot, double pos) {
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(150),
                                new ParallelCommandGroup(
                                        new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.BUCKET_IDEAL),
                                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.BUCKET_IDEAL),
                                        new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER)
                                )
                        ),
                                new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, pos)
                )
        );
    }
}
