package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.DROPDOWN_COCKED_UP;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.DROPDOWN_EXTEND;
import static org.firstinspires.ftc.teamcode.common.robot.Globals.DROPDOWN_TRANSFER;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.robot.Globals;

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
            case INTAKE: //nothing will happen (will push to command to read slides position)
                break;
            case READY:
                dropdownLeft.setPosition(DROPDOWN_EXTEND);
                dropdownRight.setPosition(DROPDOWN_EXTEND);
                break;
            case COCKED_UP:
                dropdownLeft.setPosition(DROPDOWN_COCKED_UP);
                dropdownRight.setPosition(DROPDOWN_COCKED_UP);
                break;
        }
    }

}