package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.EXTENDO_TRANSFER_FACTOR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.WristCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class DropResetReadyCmd extends SequentialCommandGroup {
    public DropResetReadyCmd(Robot robot) {
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_RETRACT_POS),
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_DEPOSIT),
                                new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.SPECIMEN_DEPOSIT)
                        ),
                                        new WaitCommand(100),
                                        new UninterruptibleCommand(new SequentialCommandGroup(
                                                new InstantCommand(() -> {
                                            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                            robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                        }),
                                                new WaitCommand(50),
                                                new ParallelCommandGroup(
                                                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_READY_POS),
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(150),
                                                                new ParallelCommandGroup(
                                                                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER),
                                                                        new WristCommand(robot.wristSubsystem, Globals.OuttakeWristState.TRANSFER),
                                                                        new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN_TRANSFER)
                                                                )
                                                        )
                                                ))
                                        )
                )
        );
    }
}
