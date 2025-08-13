package org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.DropdownSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class DropdownCommand extends CommandBase {
    DropdownSubsystem dropdownSubsystem;
    Globals.DropdownState dropdownState;

    public DropdownCommand(DropdownSubsystem dropDownSubsystemInput, Globals.DropdownState dropDownStateInput) {
        this.dropdownSubsystem = dropDownSubsystemInput;
        this.dropdownState = dropDownStateInput;
        addRequirements(dropdownSubsystem);
    }

    @Override
    public void initialize() {
        dropdownSubsystem.update(dropdownState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
