package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.DropdownSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.GateSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.SpinnerSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeSlidesSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.WristSubsystem;
import org.firstinspires.ftc.teamcode.common.roadrunner.PinpointDrive;

import java.util.List;

public class Robot {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear; //Drivetrain motors
    public DcMotorEx leftLift, rightLift; //Outtake lift motors
    public DcMotorEx extendoMotor; //Intake extension motor
    public DcMotorEx intakeSpinner; //Intake spinner motor

    public ServoImplEx intakeCoaxialLeft, intakeCoaxialRight; //Dropdown
    public ServoImplEx gate; //Micro for intake gate
    public ServoImplEx outtakeClaw, outtakeWrist, leftOuttakeArm, rightOuttakeArm; //All outtake

    public NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;

    public double voltage;
    public DriveSubsystem driveSubsystem;
    public DropdownSubsystem dropdownSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public GateSubsystem gateSubsystem;
    public SpinnerSubsystem spinnerSubsystem;
    public ClawSubsystem clawSubsystem;
    public OuttakeArmSubsystem outtakeArmSubsystem;
    public OuttakeSlidesSubsystem outtakeSlidesSubsystem;
    public WristSubsystem wristSubsystem;
    public PinpointDrive pinPointDrive;
    List<LynxModule> allHubs;

    public Robot(HardwareMap hardwareMap, Pose2d initialPose, boolean autoBoolean) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");
        intakeSpinner = hardwareMap.get(DcMotorEx.class, "spinnerMotor");
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        //Reversing motors:
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        //TODO: reverse all the correct motors
        rightLift.setDirection(DcMotorEx.Direction.FORWARD);
        leftLift.setDirection(DcMotorEx.Direction.FORWARD);
        extendoMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeSpinner.setDirection(DcMotorEx.Direction.FORWARD);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (autoBoolean) {
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        intakeCoaxialLeft = hardwareMap.get(ServoImplEx.class, "intakeCoaxialLeft");
        intakeCoaxialRight = hardwareMap.get(ServoImplEx.class, "intakeCoaxialRight");
        gate = hardwareMap.get(ServoImplEx.class, "gate");
        outtakeClaw = hardwareMap.get(ServoImplEx.class, "outtakeClaw");
        outtakeWrist = hardwareMap.get(ServoImplEx.class, "outtakeWrist");
        leftOuttakeArm = hardwareMap.get(ServoImplEx.class, "leftOuttakeArm");
        rightOuttakeArm = hardwareMap.get(ServoImplEx.class, "rightOuttakeArm");

        intakeCoaxialRight.setDirection(Servo.Direction.REVERSE);
        rightOuttakeArm.setDirection(Servo.Direction.REVERSE);

        pinPointDrive = new PinpointDrive(hardwareMap, initialPose);
        driveSubsystem = new DriveSubsystem(pinPointDrive, false);
        dropdownSubsystem = new DropdownSubsystem(intakeCoaxialLeft, intakeCoaxialRight);
        extendoSubsystem = new ExtendoSubsystem(extendoMotor);
        gateSubsystem = new GateSubsystem(gate);
        spinnerSubsystem = new SpinnerSubsystem(intakeSpinner);
        clawSubsystem = new ClawSubsystem(outtakeClaw);
        outtakeArmSubsystem = new OuttakeArmSubsystem(leftOuttakeArm, rightOuttakeArm);
        outtakeSlidesSubsystem = new OuttakeSlidesSubsystem(leftLift, rightLift);
        wristSubsystem = new WristSubsystem(outtakeWrist);

        CommandScheduler.getInstance().registerSubsystem(
                driveSubsystem,
                dropdownSubsystem,
                extendoSubsystem,
                gateSubsystem,
                spinnerSubsystem,
                clawSubsystem,
                outtakeArmSubsystem,
                outtakeSlidesSubsystem,
                wristSubsystem
        );
    }

    public float[] getColorOfSample() {
        return new float[] { colorSensor.getNormalizedColors().red, colorSensor.getNormalizedColors().green, colorSensor.getNormalizedColors().blue };
    }

    public void clearCache() {
        for (LynxModule hub : allHubs) {
            if (hub.getDeviceName().equals("Servo Hub") || hub.getDeviceName().equals("Servo Hub 4") || hub.getDeviceName().equals("pinpoint"))
                return;
            hub.clearBulkCache();
        }
    }
}
