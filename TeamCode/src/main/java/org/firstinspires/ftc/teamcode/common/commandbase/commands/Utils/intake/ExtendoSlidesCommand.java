package org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class ExtendoSlidesCommand extends CommandBase {
    ExtendoSubsystem extendoSubsystem;
    double extendoLength;

    public ExtendoSlidesCommand(ExtendoSubsystem extendoSubsystemInput, double extendoInput) {
        this.extendoSubsystem = extendoSubsystemInput;
        this.extendoLength = extendoInput;
        addRequirements(extendoSubsystem);
    }

    @Override
    public void initialize() {
        extendoSubsystem.extendoSetPosition(extendoLength);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(extendoSubsystem.extendoMotor.getCurrentPosition() - extendoLength) < Globals.EXTENDO_MAX_TOLERANCE);
    }
}
