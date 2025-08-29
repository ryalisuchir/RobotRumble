package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.DropdownSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.GateSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.SpinnerSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeSlidesSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.WristSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.util.List;

public class Robot {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear; //Drivetrain motors
    public DcMotorEx leftLift, rightLift; //Outtake lift motors
    public DcMotorEx extendoMotor; //Intake extension motor
    public DcMotorEx intakeSpinner; //Intake spinner motor

    public DropdownSubsystem dropdownSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public GateSubsystem gateSubsystem;
    public SpinnerSubsystem spinnerSubsystem;
    public ClawSubsystem clawSubsystem;
    public OuttakeArmSubsystem outtakeArmSubsystem;
    public OuttakeSlidesSubsystem outtakeSlidesSubsystem;
    public WristSubsystem wristSubsystem;


    public ServoImplEx intakeCoaxialLeft, intakeCoaxialRight; //Dropdown
    public ServoImplEx gate; //Micro for intake gate
    public ServoImplEx outtakeClaw, outtakeWrist, outtakeArm; //All outtake
    public ServoImplEx leftPto, rightPto; //PTO Hang
    public CRServo leftHang, rightHang; //Level 2 Hang

    public List<LynxModule> allHubs;
    public LynxModule ControlHub;
    public RevColorSensorV3 colorSensor;

    public DriveSubsystem driveSubsystem;
    public PinpointDrive pinPointDrive;


    public Robot(HardwareMap hardwareMap, Pose2d initialPose, boolean autoBoolean, boolean blue) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");
        intakeSpinner = hardwareMap.get(DcMotorEx.class, "spinnerMotor");

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                ControlHub = hub;
            }

        }

        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        //TODO: reverse all the correct motors
        rightLift.setDirection(DcMotorEx.Direction.FORWARD);
        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        extendoMotor.setDirection(DcMotorEx.Direction.REVERSE);
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

        if (blue) {
            Globals.allianceColor = Globals.AllianceColor.BLUE;
        } else {
            Globals.allianceColor = Globals.AllianceColor.RED;
        }

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);

        intakeCoaxialLeft = hardwareMap.get(ServoImplEx.class, "intakeCoaxialLeft");
        intakeCoaxialRight = hardwareMap.get(ServoImplEx.class, "intakeCoaxialRight");
        gate = hardwareMap.get(ServoImplEx.class, "gate");
        outtakeClaw = hardwareMap.get(ServoImplEx.class, "outtakeClaw");
        outtakeWrist = hardwareMap.get(ServoImplEx.class, "outtakeWrist");
        outtakeArm = hardwareMap.get(ServoImplEx.class, "outtakeArm");
        leftHang = hardwareMap.get(CRServo.class, "leftHang");
        rightHang = hardwareMap.get(CRServo.class, "rightHang");
        leftPto = hardwareMap.get(ServoImplEx.class, "leftPto");
        rightPto = hardwareMap.get(ServoImplEx.class, "rightPto");

        intakeCoaxialRight.setDirection(Servo.Direction.REVERSE);

        pinPointDrive = new PinpointDrive(hardwareMap, initialPose);
        driveSubsystem = new DriveSubsystem(pinPointDrive, false);
        dropdownSubsystem = new DropdownSubsystem(intakeCoaxialLeft, intakeCoaxialRight);
        extendoSubsystem = new ExtendoSubsystem(extendoMotor);
        gateSubsystem = new GateSubsystem(gate);
        spinnerSubsystem = new SpinnerSubsystem(intakeSpinner);
        clawSubsystem = new ClawSubsystem(outtakeClaw);
        outtakeArmSubsystem = new OuttakeArmSubsystem(outtakeArm);
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

    public void clearCache() {
        for (LynxModule hub : allHubs) {
            if (hub.getDeviceName().equals("Servo Hub") || hub.getDeviceName().equals("Servo Hub 4") || hub.getDeviceName().equals("pinpoint"))
                return;
            hub.clearBulkCache();
        }
    }

}
