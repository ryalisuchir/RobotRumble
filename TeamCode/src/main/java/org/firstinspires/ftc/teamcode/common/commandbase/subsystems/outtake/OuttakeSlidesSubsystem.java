package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.robot.Globals;

@Config
public class OuttakeSlidesSubsystem extends SubsystemBase {
    public static double p = 0.015;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    private double lastPower = 0.0;
    private static final PIDFController slidePIDF = new PIDFController(p, i, d, f);
    public static double setPoint = 0;
    public final DcMotorEx leftLift, rightLift;
    public ElapsedTime timer = new ElapsedTime();
    int motorPosition;
    Globals.OuttakeSlidesState outtakeSlidesState;

    public OuttakeSlidesSubsystem(DcMotorEx depoLeftInput, DcMotorEx depoRightInput) {
        leftLift = depoLeftInput;
        rightLift = depoRightInput;
    }

    public void outtakeSlidesLoop() {
        timer.reset();

        motorPosition = rightLift.getCurrentPosition();

            slidePIDF.setP(p);
            slidePIDF.setI(i);
            slidePIDF.setD(d);
            slidePIDF.setF(f);

            slidePIDF.setSetPoint(setPoint);

            double power = Range.clip(slidePIDF.calculate(motorPosition, setPoint), -1, 1);

        if (Math.abs(power - lastPower) > 0.03) {
            leftLift.setPower(power);
            rightLift.setPower(power);
            lastPower = power;
        }
    }

    public void outtakeSetPosition(double customSlidesPosition) {
        setPoint = customSlidesPosition;
        if (customSlidesPosition > rightLift.getCurrentPosition()) {
            outtakeSlidesState = Globals.OuttakeSlidesState.EXTENDING;
        } else if (customSlidesPosition < rightLift.getCurrentPosition()) {
            outtakeSlidesState = Globals.OuttakeSlidesState.RETRACTING;
        } else if (customSlidesPosition == rightLift.getCurrentPosition()) {
            outtakeSlidesState = Globals.OuttakeSlidesState.REST;
        }
    }
}