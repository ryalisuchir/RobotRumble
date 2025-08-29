package org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeArmSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

public class OuttakeArmCommand extends CommandBase {
    OuttakeArmSubsystem outtakeArmSubsystem;
    Globals.OuttakeArmState outtakeArmState;
    private final ElapsedTime timer = new ElapsedTime();

    public OuttakeArmCommand(OuttakeArmSubsystem outtakeArmSubsystemInput, Globals.OuttakeArmState outtakeArmStateInput) {
        this.outtakeArmSubsystem = outtakeArmSubsystemInput;
        this.outtakeArmState = outtakeArmStateInput;
    }

    @Override
    public void initialize() {
        outtakeArmSubsystem.update(outtakeArmState);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= Globals.SERVO_WAIT_TIME;
    }
}
