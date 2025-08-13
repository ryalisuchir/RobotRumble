package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.robot.Globals;

@Config
public class OuttakeArmSubsystem extends SubsystemBase {
    public final ServoImplEx outtakeArm;
    public Globals.OuttakeArmState armState = Globals.OuttakeArmState.SPECIMEN_GRAB;

    public OuttakeArmSubsystem(ServoImplEx outtakeArmInput) {
        outtakeArm = outtakeArmInput;
    }

    public void update(Globals.OuttakeArmState outtakeArmStateInput) {
        armState = outtakeArmStateInput; //Sets global position to the new outtake
        switch (outtakeArmStateInput) {
            case TRANSFER:
                outtakeArm.setPosition(Globals.OUTTAKE_ARM_TRANSFER);
                break;
            case BUCKET_IDEAL:
                outtakeArm.setPosition(Globals.OUTTAKE_ARM_BUCKET);
                break;
            case SPECIMEN_GRAB:
                outtakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_GRAB);
                break;
            case SPECIMEN_DEPOSIT:
                outtakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_DEPOSIT);
                break;
        }
    }

}