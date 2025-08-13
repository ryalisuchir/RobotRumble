package org.firstinspires.ftc.teamcode.common.commandbase.commands.Intake;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.MAX_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.MIN_DISTANCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.opmode.Testing.ColorSensorHSV.detectSampleColor;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.Outtake.OuttakeDepositReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.common.robot.ColorUtils;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

import java.util.Set;

public class ExtendAndSpinCommand extends CommandBase {
        public enum SampleColorDetected {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    private final Robot robot;
    private final Set<SampleColorDetected> targetColors;
    private final double extendoTargetPosition;
    private boolean colorDetected = false;

    public ExtendAndSpinCommand(
            Robot robot,
            Set<SampleColorDetected> targetColors,
            double extendoTargetPosition
    ) {
        this.targetColors = targetColors;
        this.extendoTargetPosition = extendoTargetPosition;
        this.robot = robot;

        addRequirements(robot.extendoSubsystem, robot.spinnerSubsystem, robot.gateSubsystem);
    }

    @Override
    public void initialize() {
        colorDetected = false;
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new ExtendoSlidesCommand(robot.extendoSubsystem, extendoTargetPosition),
                        new DropdownCommand(robot.dropdownSubsystem, Globals.DropdownState.INTAKE),
                        new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.INTAKING),
                        new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED)

                )
        );
    }

    @Override
    public void execute() {
        if (colorDetected) return;  // no need to re-check once detected
        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);
        if (distance > MIN_DISTANCE_THRESHOLD && distance < MAX_DISTANCE_THRESHOLD) {
            float[] hsv = new float[3];
            int r = robot.colorSensor.red();
            int g = robot.colorSensor.green();
            int b = robot.colorSensor.blue();

            // Normalize and convert to HSV
            Color.RGBToHSV(r * 8, g * 8, b * 8, hsv); // Scale to 0–255

            float hue = hsv[0];
            float sat = hsv[1];
            float val = hsv[2];

            ExtendAndSpinCommand.SampleColorDetected colorDetectedR = detectSampleColor(hue, sat, val);

            if (targetColors.contains(colorDetectedR)) {
                colorDetected = true;
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED),
                                new DropdownCommand(robot.dropdownSubsystem, Globals.DropdownState.TRANSFER),
                                new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION)
                        ),
                                new ParallelCommandGroup(
                                        new GateCommand(robot.gateSubsystem, Globals.GateState.OPEN_TRANSFER),
                                        new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_POS)
                                ),
                                new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.CLOSED),
                                new WaitCommand(500),
                                new OuttakeDepositReadyCommand(robot)
                        )
                );
                cancel();
            } else {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new GateCommand(robot.gateSubsystem, Globals.GateState.OPEN_EJECT),
                                new WaitCommand(900),
                                new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED)
                        )
                );
            }
        }
    }

    @Override
    public boolean isFinished() {
        return colorDetected;
    }

    @Override
    public void end(boolean interrupted) {
        robot.spinnerSubsystem.update(Globals.SpinnerState.STOPPED);
    }
}
