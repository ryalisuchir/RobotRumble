package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.DROPDOWN_EXTEND;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.DROPDOWN_GROUND;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.DROPDOWN_TRANSFER;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

@Config
public class DropdownSubsystem extends SubsystemBase {

    public final ServoImplEx dropdownLeft, dropdownRight;
    public Globals.DropdownState dropdownState = Globals.DropdownState.TRANSFER;

    public DropdownSubsystem(ServoImplEx dropdownLeftInput, ServoImplEx dropdownRightInput) {
        dropdownLeft = dropdownLeftInput;
        dropdownRight = dropdownRightInput;
    }

    public void update(Globals.DropdownState dropdownStateInput) {
        dropdownState = dropdownStateInput;
        switch (dropdownStateInput) {
            case TRANSFER:
                dropdownLeft.setPosition(DROPDOWN_TRANSFER);
                dropdownRight.setPosition(DROPDOWN_TRANSFER);
                break;
            case INTAKE:
                dropdownLeft.setPosition(0);
                dropdownRight.setPosition(0);
                break;
            case READY:
                dropdownLeft.setPosition(DROPDOWN_EXTEND);
                dropdownRight.setPosition(DROPDOWN_EXTEND);
                break;
        }
    }

}