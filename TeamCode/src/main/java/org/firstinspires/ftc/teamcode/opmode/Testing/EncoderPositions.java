package org.firstinspires.ftc.teamcode.opmode.Testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.robot.Robot;

@Autonomous
public class EncoderPositions extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)), true, true);
        telemetry.addLine("Reset all encoders.");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("Motor Encoders:");
        telemetry.addData("leftLift: ", robot.leftLift.getCurrentPosition());
        telemetry.addData("rightLift: ", robot.rightLift.getCurrentPosition());
        telemetry.addData("extendoMotor: ", robot.extendoMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Closed OpMode.");
    }
}
