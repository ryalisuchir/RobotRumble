package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.robot.Globals;

@Config
public class WristSubsystem extends SubsystemBase {

    public final ServoImplEx wrist;
    public Globals.OuttakeWristState wristState = Globals.OuttakeWristState.SPECIMEN_GRAB;

    public WristSubsystem(ServoImplEx wristInput) {
        wrist = wristInput;
    }

    public void update(Globals.OuttakeWristState wristStateInput) {
        wristState = wristStateInput;
        switch (wristStateInput) {
            case SPECIMEN_GRAB:
                wrist.setPosition(Globals.OUTTAKE_WRIST_SPECIMEN_GRAB);
                break;
            case TRANSFER:
                wrist.setPosition(Globals.OUTTAKE_WRIST_TRANSFER);
                break;
            case SPECIMEN_DEPOSIT:
                wrist.setPosition(Globals.OUTTAKE_WRIST_SPECIMEN_DEPOSIT);
                break;
            case BUCKET_IDEAL:
                wrist.setPosition(Globals.OUTTAKE_WRIST_BUCKET);
                break;
        }
    }
}