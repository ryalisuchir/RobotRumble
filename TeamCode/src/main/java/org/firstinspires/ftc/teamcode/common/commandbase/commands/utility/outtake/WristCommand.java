package org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.WristSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class WristCommand extends CommandBase {
    WristSubsystem wristSubsystem;
    Globals.OuttakeWristState wristState;
    private final ElapsedTime timer = new ElapsedTime();

    public WristCommand(WristSubsystem wristSubsystemInput, Globals.OuttakeWristState wristStateInput) {
        this.wristSubsystem = wristSubsystemInput;
        this.wristState = wristStateInput;
    }

    @Override
    public void initialize() {
        wristSubsystem.update(wristState);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= Globals.SERVO_WAIT_TIME;
    }
}
