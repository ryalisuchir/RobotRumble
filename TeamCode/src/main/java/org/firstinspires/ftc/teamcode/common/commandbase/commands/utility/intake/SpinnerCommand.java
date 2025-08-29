package org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.SpinnerSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class SpinnerCommand extends CommandBase {
    SpinnerSubsystem spinnerSubsystem;
    Globals.SpinnerState spinnerState;

    public SpinnerCommand(SpinnerSubsystem spinnerSubsystemInput, Globals.SpinnerState spinnerStateInput) {
        this.spinnerSubsystem = spinnerSubsystemInput;
        this.spinnerState = spinnerStateInput;
    }

    @Override
    public void initialize() {
        spinnerSubsystem.update(spinnerState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
