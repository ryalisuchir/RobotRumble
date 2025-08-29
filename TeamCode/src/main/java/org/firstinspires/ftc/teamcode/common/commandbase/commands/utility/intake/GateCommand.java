package org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.GateSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class GateCommand extends CommandBase {
    GateSubsystem gateSubsystem;
    Globals.GateState gateState;
    private final ElapsedTime timer = new ElapsedTime();

    public GateCommand(GateSubsystem gateSubsystemInput, Globals.GateState gateStateInput) {
        this.gateSubsystem = gateSubsystemInput;
        this.gateState = gateStateInput;
    }

    @Override
    public void initialize() {
        gateSubsystem.update(gateState);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= Globals.SERVO_WAIT_TIME;
    }
}

