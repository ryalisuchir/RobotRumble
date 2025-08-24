package org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.MAX_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.MIN_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.opmode.Testing.ColorSensorHSV.detectSampleColor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.sun.tools.javac.util.List;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

import java.util.Set;

public class ExtendAndDontSpinCommand extends CommandBase {

    private boolean resetDone = false;

    private final Robot robot;
    private final double extendoTargetPosition;
    public static boolean colorDetected = false;

    public ExtendAndDontSpinCommand(
            Robot robot,
            Set<ExtendAndSpinCommand.SampleColorDetected> targetColors,
            double extendoTargetPosition
    ) {
        this.extendoTargetPosition = extendoTargetPosition;
        this.robot = robot;

//        addRequirements(robot.extendoSubsystem, robot.spinnerSubsystem, robot.gateSubsystem); i um don't like solverslib anymore
    }

    @Override
    public void initialize() {
        colorDetected = false;
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ExtendoSlidesCommand(robot.extendoSubsystem, extendoTargetPosition),
                                new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.INTAKE)
                        ),
                        new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED)

                )
        );
    }

    @Override
    public void execute() {
        if (!resetDone) {
            CommandScheduler.getInstance().schedule(
                    new UninterruptibleCommand(
                            new SequentialCommandGroup(
                                    new WaitCommand(500),
                                    new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_RETRACT_POS),
                                    new WaitCommand(100),
                                    new InstantCommand(() -> {
                                        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    }),
                                    new DeferredCommand(() -> new InstantCommand(() -> resetDone = true), List.nil())
                            )
                    )
            );
        }
    }


    @Override
    public boolean isFinished() {
        return resetDone;
    }

    @Override
    public void end(boolean interrupted) {
        resetDone = false;
    }
}
