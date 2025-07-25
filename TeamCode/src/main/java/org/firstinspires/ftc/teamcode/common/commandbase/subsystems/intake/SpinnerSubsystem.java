package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.robot.Globals;

@Config
public class SpinnerSubsystem extends SubsystemBase {
    public final DcMotorEx spinnerMotor;
    public Globals.SpinnerState spinnerState = Globals.SpinnerState.STOPPED;

    public SpinnerSubsystem(DcMotorEx spinnerMotorInput) {
        spinnerMotor = spinnerMotorInput;
    }

    public void update(Globals.SpinnerState spinnerStateInput) {
        spinnerState = spinnerStateInput;
        switch (spinnerStateInput) {
            case STOPPED:
                spinnerMotor.setPower(0);
                break;
            case INTAKING:
                spinnerMotor.setPower(1);
                break;
            case EJECTING:
                spinnerMotor.setPower(-0.7);
                break;
        }
    }
}