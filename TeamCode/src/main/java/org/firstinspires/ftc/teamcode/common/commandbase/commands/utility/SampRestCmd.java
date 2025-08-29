package org.firstinspires.ftc.teamcode.common.commandbase.commands.utility;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class SampRestCmd extends ParallelCommandGroup {
    public SampRestCmd(Robot robot) {
        addCommands(
               new ParallelCommandGroup(
                       new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SAMP_REST),
                       new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.SAMP_REST),
                       new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
                       new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER),
                       new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED)
               )
        );
    }
}
