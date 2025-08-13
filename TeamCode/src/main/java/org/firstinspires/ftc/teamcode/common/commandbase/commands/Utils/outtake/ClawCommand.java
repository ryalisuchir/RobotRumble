package org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class ClawCommand extends CommandBase {
    ClawSubsystem clawSubsystem;
    Globals.OuttakeClawState clawState;

    public ClawCommand(ClawSubsystem clawSubsystemInput, Globals.OuttakeClawState clawStateInput) {
        this.clawSubsystem = clawSubsystemInput;
        this.clawState = clawStateInput;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.update(clawState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
