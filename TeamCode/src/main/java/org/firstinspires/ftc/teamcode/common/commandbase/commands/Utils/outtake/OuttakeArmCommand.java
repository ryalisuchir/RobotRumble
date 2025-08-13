package org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeArmSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class OuttakeArmCommand extends CommandBase {
    OuttakeArmSubsystem outtakeArmSubsystem;
    Globals.OuttakeArmState outtakeArmState;

    public OuttakeArmCommand(OuttakeArmSubsystem outtakeArmSubsystemInput, Globals.OuttakeArmState outtakeArmStateInput) {
        this.outtakeArmSubsystem = outtakeArmSubsystemInput;
        this.outtakeArmState = outtakeArmStateInput;
        addRequirements(outtakeArmSubsystem);
    }

    @Override
    public void initialize() {
        outtakeArmSubsystem.update(outtakeArmState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
