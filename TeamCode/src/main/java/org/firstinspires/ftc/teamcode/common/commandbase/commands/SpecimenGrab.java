package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class SpecimenGrab extends SequentialCommandGroup {
    public SpecimenGrab(Robot robot) {
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
