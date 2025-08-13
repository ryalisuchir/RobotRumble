package org.firstinspires.ftc.teamcode.opmode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class GearPositions extends OpMode {

    public static double coaxIntake = 0.5;
    public static double gate = 0.5;
    public static double arm = 0.5;
    public static double wrist = 0.5;
    public static double claw = 0.5;
    public static double spinPower = 0;

    public Servo coax1, coax2, blocker, arm1, wrist1, claw1;
    public DcMotorEx spinner;

    @Override
    public void init() {
        coax1 = hardwareMap.get(Servo.class, "intakeCoaxialLeft");
        coax2 = hardwareMap.get(Servo.class, "intakeCoaxialRight");
        blocker = hardwareMap.get(Servo.class, "gate");
        arm1 = hardwareMap.get(Servo.class, "outtakeArm");
        wrist1 = hardwareMap.get(Servo.class, "outtakeWrist");
        claw1 = hardwareMap.get(Servo.class, "outtakeClaw");
        spinner = hardwareMap.get(DcMotorEx.class, "spinnerMotor");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        coax2.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void loop() {
        coax1.setPosition(coaxIntake);
        coax2.setPosition(coaxIntake);
        blocker.setPosition(gate);
        spinner.setPower(spinPower);
        arm1.setPosition(arm);
        wrist1.setPosition(wrist);
        claw1.setPosition(claw);

        telemetry.update();
    }
}
