package org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.DropdownSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class DropdownCommand extends CommandBase {
    DropdownSubsystem dropdownSubsystem;
    Globals.DropdownState dropdownState;
    Robot robot;

    public double getOptimalPosition(double currentPosition, double extendoMaxRetraction, double extendoFull) {
        if (currentPosition > Globals.EXTENDO_MAX_EXTENSION-50) {
            return 0.53;
        } else {
            return 0.45;
        }
    }

    public DropdownCommand(Robot robot, DropdownSubsystem dropDownSubsystemInput, Globals.DropdownState dropDownStateInput) {
        this.dropdownSubsystem = dropDownSubsystemInput;
        this.dropdownState = dropDownStateInput;
        this.robot = robot;
        addRequirements(dropdownSubsystem);
    }

    @Override
    public void initialize() {

        if (dropdownState == Globals.DropdownState.INTAKE) {
            Globals.dropdownState = Globals.DropdownState.INTAKE;
            dropdownSubsystem.dropdownLeft.setPosition(getOptimalPosition(robot.extendoMotor.getCurrentPosition(), Globals.EXTENDO_MAX_RETRACTION, Globals.EXTENDO_MAX_EXTENSION));
            dropdownSubsystem.dropdownRight.setPosition(getOptimalPosition(robot.extendoMotor.getCurrentPosition(), Globals.EXTENDO_MAX_RETRACTION, Globals.EXTENDO_MAX_EXTENSION));
        } else {
            dropdownSubsystem.update(dropdownState);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
