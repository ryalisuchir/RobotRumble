package org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake;

import static org.firstinspires.ftc.teamcode.common.robot.Globals.EXTENDO_TRANSFER_FACTOR;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class ExtendoSlidesCommand extends CommandBase {
    ExtendoSubsystem extendoSubsystem;
    double extendoLength;

    public ExtendoSlidesCommand(ExtendoSubsystem extendoSubsystemInput, double extendoInput) {
        this.extendoSubsystem = extendoSubsystemInput;
        this.extendoLength = extendoInput;
    }

    @Override
    public void initialize() {
        extendoSubsystem.extendoSetPosition(extendoLength);
    }

    @Override
    public boolean isFinished() {
        if (extendoLength == Globals.EXTENDO_MAX_EXTENSION*EXTENDO_TRANSFER_FACTOR) {
            return true;
        } else {
            return (Math.abs(extendoSubsystem.extendoMotor.getCurrentPosition() - extendoLength) < Globals.EXTENDO_MAX_TOLERANCE);
        }
    }
}
