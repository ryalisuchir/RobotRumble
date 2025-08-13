package org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.GateSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class GateCommand extends CommandBase {
    GateSubsystem gateSubsystem;
    Globals.GateState gateState;

    public GateCommand(GateSubsystem gateSubsystemInput, Globals.GateState gateStateInput) {
        this.gateSubsystem = gateSubsystemInput;
        this.gateState = gateStateInput;
        addRequirements(gateSubsystem);
    }

    @Override
    public void initialize() {
        gateSubsystem.update(gateState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
