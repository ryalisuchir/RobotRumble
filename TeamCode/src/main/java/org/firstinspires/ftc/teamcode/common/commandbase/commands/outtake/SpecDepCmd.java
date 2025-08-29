package org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class SpecDepCmd extends SequentialCommandGroup {
    public SpecDepCmd(Robot robot) {
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_POS),
                                new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN_TRANSFER),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_GRAB),
                                                new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.SPECIMEN_GRAB)
                                        )
                                )
                        ),
                        new UninterruptibleCommand(
                                new SequentialCommandGroup(
                                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_RETRACT_POS),
                                        new WaitCommand(100),
                                        new InstantCommand(() -> {
                                            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                            robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                        })
                                )
                        )
                )
        );
    }
}
