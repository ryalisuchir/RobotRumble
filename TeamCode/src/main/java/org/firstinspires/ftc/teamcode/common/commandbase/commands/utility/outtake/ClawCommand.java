package org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class ClawCommand extends CommandBase {
    ClawSubsystem clawSubsystem;
    Globals.OuttakeClawState clawState;
    private final ElapsedTime timer = new ElapsedTime();

    public ClawCommand(ClawSubsystem clawSubsystemInput, Globals.OuttakeClawState clawStateInput) {
        this.clawSubsystem = clawSubsystemInput;
        this.clawState = clawStateInput;
    }

    @Override
    public void initialize() {
        clawSubsystem.update(clawState);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= Globals.SERVO_WAIT_TIME;
    }
}
