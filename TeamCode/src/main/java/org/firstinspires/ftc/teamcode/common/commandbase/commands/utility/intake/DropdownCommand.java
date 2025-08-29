package org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.DROPDOWN_VAL;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.EXTENDO_POS;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.DropdownSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

public class DropdownCommand extends CommandBase {
    DropdownSubsystem dropdownSubsystem;
    Globals.DropdownState dropdownState;
    private final ElapsedTime timer = new ElapsedTime();
    Robot robot;

    public static double getDropdown() {
        if (ExtendoSubsystem.setPoint <= EXTENDO_POS[0]) return DROPDOWN_VAL[0];
        if (ExtendoSubsystem.setPoint >= EXTENDO_POS[EXTENDO_POS.length - 1]) return DROPDOWN_VAL[DROPDOWN_VAL.length - 1];

        for (int i = 0; i < EXTENDO_POS.length - 1; i++) {
            if (ExtendoSubsystem.setPoint >= EXTENDO_POS[i] && ExtendoSubsystem.setPoint <= EXTENDO_POS[i + 1]) {
                double t = (ExtendoSubsystem.setPoint - EXTENDO_POS[i]) / (EXTENDO_POS[i + 1] - EXTENDO_POS[i]);
                return DROPDOWN_VAL[i] + t * (DROPDOWN_VAL[i + 1] - DROPDOWN_VAL[i]);
            }
        }
        return DROPDOWN_VAL[0];
    }

    public DropdownCommand(Robot robot, DropdownSubsystem dropDownSubsystemInput, Globals.DropdownState dropDownStateInput) {
        this.dropdownSubsystem = dropDownSubsystemInput;
        this.dropdownState = dropDownStateInput;
        this.robot = robot;
    }

    @Override
    public void initialize() {
        timer.reset();
        if (dropdownState == Globals.DropdownState.INTAKE) {
            Globals.dropdownState = Globals.DropdownState.INTAKE;
            dropdownSubsystem.dropdownLeft.setPosition(getDropdown());
            dropdownSubsystem.dropdownRight.setPosition(getDropdown());
        } else {
            dropdownSubsystem.update(dropdownState);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= Globals.SERVO_WAIT_TIME;
    }
}
