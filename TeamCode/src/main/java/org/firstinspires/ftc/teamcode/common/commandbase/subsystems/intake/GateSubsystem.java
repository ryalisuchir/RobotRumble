package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.robot.Globals;

@Config
public class GateSubsystem extends SubsystemBase {
    public final ServoImplEx gateServo;
    public Globals.GateState gateState = Globals.GateState.CLOSED;

    public GateSubsystem(ServoImplEx gateServoInput) {
        gateServo = gateServoInput;
    }

    public void update(Globals.GateState gateStateInput) {
        gateState = gateStateInput;
        switch (gateStateInput) {
            case OPEN:
                gateServo.setPosition(Globals.GATE_OPEN);
                break;
            case CLOSED:
                gateServo.setPosition(Globals.GATE_CLOSED);
                break;
        }
    }
}