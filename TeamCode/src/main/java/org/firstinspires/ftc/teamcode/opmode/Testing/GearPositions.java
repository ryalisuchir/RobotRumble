package org.firstinspires.ftc.teamcode.opmode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

@Config
@Autonomous
public class GearPositions extends OpMode {
    Robot robot;
    public static double coaxIntake = Globals.DROPDOWN_TRANSFER;
    public static double gate = Globals.GATE_CLOSED;
    public static double arm = Globals.OUTTAKE_ARM_TRANSFER;
    public static double wrist = Globals.OUTTAKE_WRIST_TRANSFER;
    public static double claw = Globals.OUTTAKE_CLAW_TRANSFER_OPEN;
    public static double spinPower = 0;

    public Servo coax1, coax2, blocker, arm1, wrist1, claw1;
    public DcMotorEx spinner;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)), true, true);
        telemetry.addLine("Reset all encoders.");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.intakeCoaxialLeft.setPosition(coaxIntake);
        robot.intakeCoaxialRight.setPosition(coaxIntake);
        robot.gate.setPosition(gate);
        robot.intakeSpinner.setPower(spinPower);
        robot.outtakeArm.setPosition(0);
        robot.outtakeWrist.setPosition(1);
        robot.outtakeClaw.setPosition(claw);

        telemetry.addLine("Encoders:");
        telemetry.addData("leftLift: ", robot.leftLift.getCurrentPosition());
        telemetry.addData("rightLift: ", robot.rightLift.getCurrentPosition());
        telemetry.addData("extendoMotor: ", robot.extendoMotor.getCurrentPosition());

        telemetry.update();
    }
}
