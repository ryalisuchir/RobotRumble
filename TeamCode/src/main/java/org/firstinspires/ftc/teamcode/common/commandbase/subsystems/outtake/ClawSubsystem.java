package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.robot.Globals;

@Config
public class ClawSubsystem extends SubsystemBase {

    public final ServoImplEx claw;
    public Globals.OuttakeClawState clawState = Globals.OuttakeClawState.CLOSED;

    public ClawSubsystem(ServoImplEx clawInput) {
        claw = clawInput;
    }

    public void update(Globals.OuttakeClawState clawInputState) {
        clawState = clawInputState;
        switch (clawInputState) {
            case OPEN:
                claw.setPosition(Globals.OUTTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                claw.setPosition(Globals.OUTTAKE_CLAW_CLOSE);
                break;
            case OPEN_TRANSFER:
                claw.setPosition(Globals.OUTTAKE_CLAW_TRANSFER_OPEN);
                break;
        }
    }
}