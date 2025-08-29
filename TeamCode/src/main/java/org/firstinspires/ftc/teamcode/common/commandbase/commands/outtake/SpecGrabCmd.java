package org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class SpecGrabCmd extends SequentialCommandGroup {
    public SpecGrabCmd(Robot robot) {
        addCommands(
                new SequentialCommandGroup(
                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_SPECIMEN_DEPOSIT_POS),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_DEPOSIT),
                                        new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.SPECIMEN_DEPOSIT)
                                )
                        )
                )
        );
    }
}
