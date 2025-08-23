package org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.MAX_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.MIN_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.opmode.Testing.ColorSensorHSV.detectSampleColor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
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

public class SpecSpitter extends CommandBase {
    public enum SampleColorDetected {
        RED,
        BLUE,
        YELLOW,
        NONE
    }
    private boolean resetDone = false;

    private final Robot robot;
    private final double extendoTargetPosition;
    private boolean colorDetected = false;
    int lastEncoderPos = 0;
    long lastEncoderUpdateTime = 0;
    long STALL_THRESHOLD_MS = 700;

    public SpecSpitter(
            Robot robot,
            double extendoTargetPosition
    ) {
        this.extendoTargetPosition = extendoTargetPosition;
        this.robot = robot;
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
                        new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.INTAKING),
                        new GateCommand(robot.gateSubsystem, Globals.GateState.MEGA_CLOSED)

                )
        );
    }

    @Override
    public void execute() {

        if (colorDetected) return;
        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        if (distance > MIN_DISTANCE_THRESHOLD && distance < MAX_DISTANCE_THRESHOLD || robot.intakeSpinner.getCurrent(CurrentUnit.MILLIAMPS) > 4700) {
            colorDetected = true;
            CommandScheduler.getInstance().schedule(
new ParallelCommandGroup(
        new InstantCommand(() -> robot.intakeSpinner.setPower(0.4)),
        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_EXTENSION*0.4),
        new DropdownCommand(robot, robot.dropdownSubsystem, Globals.DropdownState.TRANSFER)
)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return colorDetected;
    }

    @Override
    public void end(boolean interrupted) {
        robot.gateSubsystem.update(Globals.GateState.MEGA_CLOSED);
    }
}
