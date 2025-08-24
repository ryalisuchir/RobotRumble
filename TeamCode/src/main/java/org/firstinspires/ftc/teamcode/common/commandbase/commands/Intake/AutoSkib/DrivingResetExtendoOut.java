package org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake.AutoSkib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class DrivingResetExtendoOut extends SequentialCommandGroup {
    public DrivingResetExtendoOut(Robot robot) {
        addCommands(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new OuttakeTransferReadyCommand(robot),
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
                                        ),
                                        new OuttakeTransferReadyCommand(robot)
                                ),
                                new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED),
                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED),
                                new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER),
                                new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_EXTENSION*0.2)
                        )
        );
    }
}
