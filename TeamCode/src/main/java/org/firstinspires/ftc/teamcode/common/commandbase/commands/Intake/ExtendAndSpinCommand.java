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

public class ExtendAndSpinCommand extends CommandBase {
        public enum SampleColorDetected {
        RED,
        BLUE,
        YELLOW,
        NONE
    }
    private boolean resetDone = false;

    private final Robot robot;
    private final Set<SampleColorDetected> targetColors;
    private final double extendoTargetPosition;
    public static boolean colorDetected = false;
    int lastEncoderPos = 0;
    long lastEncoderUpdateTime = 0;
    long STALL_THRESHOLD_MS = 700;

    public ExtendAndSpinCommand(
            Robot robot,
            Set<SampleColorDetected> targetColors,
            double extendoTargetPosition
    ) {
        this.targetColors = targetColors;
        this.extendoTargetPosition = extendoTargetPosition;
        this.robot = robot;

//        addRequirements(robot.extendoSubsystem, robot.spinnerSubsystem, robot.gateSubsystem); i um don't like solverslib anymore
    }


    public static boolean goofy() {
        return colorDetected;
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
                                    new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_TRANSFER_READY_POS),
                                    new WaitCommand(500),
                                    new OuttakeSlidesCommand(robot.outtakeSlidesSubsystem, Globals.LIFT_RETRACT_POS),
                                    new WaitCommand(100),
                                    new InstantCommand(() -> {
                                        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    }),
                                    new InstantCommand(() -> resetDone = true)
                            )
                    )
            );
        }

        if (colorDetected) return;
        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

                if (robot.intakeSpinner.getCurrent(CurrentUnit.MILLIAMPS) > 6000 && robot.extendoMotor.getCurrentPosition() > Globals.EXTENDO_MAX_EXTENSION*0.5) {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.REVERSED),
                                    new WaitCommand(100),
                                    new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.INTAKING)
                            )
                    );
                    lastEncoderUpdateTime = System.currentTimeMillis();
        }


        if (distance > MIN_DISTANCE_THRESHOLD && distance < MAX_DISTANCE_THRESHOLD) {
            float[] hsv = new float[3];
            int r = robot.colorSensor.red();
            int g = robot.colorSensor.green();
            int b = robot.colorSensor.blue();
            
            Color.RGBToHSV(r * 8, g * 8, b * 8, hsv); // Scale to 0–255

            float hue = hsv[0];
            float sat = hsv[1];
            float val = hsv[2];

            ExtendAndSpinCommand.SampleColorDetected colorDetectedR = detectSampleColor(hue, sat, val);

            if (targetColors.contains(colorDetectedR)) {
                colorDetected = true;
                CommandScheduler.getInstance().schedule(
                    new TransferCommand(robot)
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
        robot.gateSubsystem.update(Globals.GateState.CLOSED);
        resetDone = false;
    }
}
