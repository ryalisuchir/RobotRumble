package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.robot.Globals;

@Config
public class ExtendoSubsystem extends SubsystemBase {
    public static double p = 0.012;
    public static double i = 0;
    public static double d = 0.00;
    public static double f = 0;
    private static final PIDFController extendoPIDF = new PIDFController(p, i, d, f);
    public static double setPoint = 0;
    private double lastPower = 0.0;
    public DcMotorEx extendoMotor;
    public ElapsedTime timer = new ElapsedTime();
    Globals.ExtendoState extendoState;
    int motorPos = 0;

    public ExtendoSubsystem(DcMotorEx extendoMotorInput) {
        extendoMotor = extendoMotorInput;
    }

    public void extendoSlidesLoop() {
        timer.reset();

        motorPos = extendoMotor.getCurrentPosition();

            extendoPIDF.setP(p);
            extendoPIDF.setI(i);
            extendoPIDF.setD(d);
            extendoPIDF.setF(f);

            extendoPIDF.setSetPoint(setPoint);

            double power = Range.clip(extendoPIDF.calculate(motorPos, setPoint), -1, 1);


            if (Math.abs(power - lastPower) > 0.03) {
                extendoMotor.setPower(power);
                lastPower = power;
            }
    }

    public void extendoSetPosition(double customSlidesPosition) {
        setPoint = customSlidesPosition;
        if (customSlidesPosition > extendoMotor.getCurrentPosition()) {
            extendoState = Globals.ExtendoState.EXTENDING;
        } else if (customSlidesPosition < extendoMotor.getCurrentPosition()) {
            extendoState = Globals.ExtendoState.RETRACTING;
        } else if (customSlidesPosition == extendoMotor.getCurrentPosition()) {
            extendoState = Globals.ExtendoState.REST;
        }
    }

}