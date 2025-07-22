package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.robot.Globals;

@Config
public class OuttakeArmSubsystem extends SubsystemBase {
    public final ServoImplEx leftOuttakeArm, rightOuttakeArm;
    public Globals.OuttakeArmState armState = Globals.OuttakeArmState.SPECIMEN_GRAB;

    public OuttakeArmSubsystem(ServoImplEx leftOuttakeArmInput, ServoImplEx rightOuttakeArmInput) {
        leftOuttakeArm = leftOuttakeArmInput;
        rightOuttakeArm = rightOuttakeArmInput;
    }

    public void update(Globals.OuttakeArmState outtakeArmStateInput) {
        armState = outtakeArmStateInput; //Sets global position to the new outtake
        switch (outtakeArmStateInput) {
            case TRANSFER:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_TRANSFER);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_TRANSFER);
                break;
            case BUCKET_IDEAL:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_BUCKET_IDEAL);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_BUCKET_IDEAL);
                break;
            case SPECIMEN_GRAB:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_GRAB);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_GRAB);
                break;
            case SPECIMEN_DEPOSIT:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_DEPOSIT);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_DEPOSIT);
                break;
        }
    }

}