package org.firstinspires.ftc.teamcode.common.commandbase.commands.Utils.outtake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeSlidesSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class OuttakeSlidesCommand extends CommandBase {
    public double depositHeight;
    OuttakeSlidesSubsystem depositSubsystem;

    public OuttakeSlidesCommand(OuttakeSlidesSubsystem depositSubsystemInput, double heightInput) {
        this.depositSubsystem = depositSubsystemInput;
        this.depositHeight = heightInput;
    }

    @Override
    public void initialize() {
        depositSubsystem.outtakeSetPosition(depositHeight);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(depositSubsystem.rightLift.getCurrentPosition() - depositHeight) < Globals.LIFT_MAX_TOLERANCE);
    }

}
