package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.GateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Set;

public class ExtendAndSpinCommand implements Command {
    Robot robot;
    private final List<String> selectedColors;

    public ExtendAndSpinCommand(Robot robot, String... colors) {
        this.robot = robot;
        this.selectedColors = Arrays.asList(colors);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_EXTENSION),
                        new DropdownCommand(robot.dropdownSubsystem, Globals.DropdownState.TRANSFER),
                        new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED),
                        new GateCommand(robot.gateSubsystem, Globals.GateState.OPEN)
                ),
                        new ParallelCommandGroup(
                                new DropdownCommand(robot.dropdownSubsystem, Globals.DropdownState.HALF),
                                new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.INTAKING)
                        )
                )
        );
    }

    @Override
    public boolean isFinished() {
        int[] rgb = robot.getColorOfSample(); //r, g, b
        float red = rgb[0];
        float green = rgb[1];
        float blue = rgb[2];

        for (String color : selectedColors) {
            switch (color.toLowerCase()) {
                case "blue":
                    if (blue >= 10 && blue <= 15 &&
                            red <= 5 && green <= 5) {
                        return true;
                    }
                    break;
                case "red":
                    if (red >= 12 && red <= 17 &&
                            green <= 6 && blue <= 6) {
                        return true;
                    }
                    break;
                case "yellow":
                    if (red >= 8 && red <= 13 &&
                            green >= 8 && green <= 13 &&
                            blue <= 4) {
                        return true;
                    }
                    break;
            }
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new GateCommand(robot.gateSubsystem, Globals.GateState.CLOSED),
                        new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED)
                        //transfer technically at this point
                )
        );
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }


}
