package org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.WristSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class WristCommand extends CommandBase {
    WristSubsystem wristSubsystem;
    Globals.OuttakeWristState wristState;

    public WristCommand(WristSubsystem wristSubsystemInput, Globals.OuttakeWristState wristStateInput) {
        this.wristSubsystem = wristSubsystemInput;
        this.wristState = wristStateInput;
    }

    @Override
    public void initialize() {
        wristSubsystem.update(wristState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
