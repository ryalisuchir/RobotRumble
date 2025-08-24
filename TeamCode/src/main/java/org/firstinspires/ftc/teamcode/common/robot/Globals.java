package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Globals {
    public static double extendoStaticMax = 15;
    public static Globals.ExtendoState extendoState = ExtendoState.REST;
    public static Globals.DropdownState dropdownState = DropdownState.TRANSFER;
    public static Globals.GateState gateState = GateState.CLOSED;
    public static Globals.SpinnerState spinnerState = SpinnerState.STOPPED;
    public static Globals.OuttakeSlidesState outtakeSlidesState = OuttakeSlidesState.REST;
    public static Globals.OuttakeClawState outtakeClawState = OuttakeClawState.CLOSED;
    public static Globals.OuttakeWristState outtakeWristState = OuttakeWristState.TRANSFER;
    public static Globals.OuttakeArmState outtakeArmState = OuttakeArmState.SPECIMEN_GRAB;

    public static Pose2d DEFAULT_START_POSE = new Pose2d(0, 0, Math.toRadians(0));
    public static Pose2d BLUE_SIDEWAYS_START_POSE = new Pose2d(41, 64, Math.toRadians(180));
    public static Pose2d BLUE_FAR_START_POSE = new Pose2d(-7, 66.43, Math.toRadians(-90));

    // Lift Subsystem Constants
    public static double LIFT_HIGH_POS = 930;
    public static double LIFT_MID_POS = 260;
    public static double LIFT_PARK_POS = 0;
    public static double LIFT_TRANSFER_POS = 80;
    public static double LIFT_TRANSFER_READY_POS = 95;
    public static double LIFT_READY_DEPOSIT_POS = 100; //after transfer, before bucket placement (maybe this is the same as mid?)
    public static double LIFT_SPECIMEN_GRAB_POS = 0;
    public static double LIFT_SPECIMEN_DEPOSIT_POS = 505;
    public static double LIFT_RETRACT_POS = 0;
    public static double LIFT_MAX_TOLERANCE = 15;

    //Extendo Subsystem Constants
    public static double EXTENDO_MAX_EXTENSION = 517;
    public static double EXTENDO_MAX_RETRACTION = 0;
    public static double EXTENDO_MAX_TOLERANCE = 15;

    public static double spinnerSpeed = 1;


    //Dropdown Subsystem Constants
    public static double DROPDOWN_TRANSFER = 0.65; //vertical
    public static double DROPDOWN_GROUND = 0.45; //good intake. 0.53 when fully extended
    public static double DROPDOWN_EXTEND = 0.65; //good over the sub

    //Gate Subsystem Constants
    public static double GATE_CLOSED = 0.09;
    public static double GATE_MEGA_CLOSE = 0;
    public static double GATE_OPEN_TRANSFER = 0.7;
    public static double GATE_OPEN_EJECT = 0.2;

    //Outtake Claw Constants
    public static double OUTTAKE_CLAW_OPEN = 0.6;
    public static double OUTTAKE_CLAW_TRANSFER_OPEN = 0.7;
    public static double OUTTAKE_CLAW_CLOSE = 0.52;

    //Outtake Wrist Constants
    public static double OUTTAKE_WRIST_SPECIMEN_GRAB = 1;
    public static double OUTTAKE_WRIST_SPECIMEN_DEPOSIT = 0.2;
    public static double OUTTAKE_WRIST_TRANSFER = 0;
    public static double OUTTAKE_WRIST_BUCKET = 0.82;

    //Outtake Arm Constants:
    public static double OUTTAKE_ARM_TRANSFER = .98;
    public static double OUTTAKE_ARM_SPECIMEN_GRAB = 0;
    public static double OUTTAKE_ARM_SPECIMEN_DEPOSIT = 0.64;
    public static double OUTTAKE_ARM_BUCKET = 0.3;

    public static double MIN_DISTANCE_THRESHOLD = 1.35;
    public static double MAX_DISTANCE_THRESHOLD = 2.5;

    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public enum AllianceColor {
        RED,
        BLUE
    }


    public enum RobotState {
        INITIALIZED,
        TRANSFERRED,
        SCORING,
        INTAKING,
        EJECTING,
        SPECIMEN_INTAKING,
        SPECIMEN_SCORING
    }
    public static RobotState robotState;

    public static OpModeType opModeType;
    public static AllianceColor allianceColor = AllianceColor.BLUE;

    //Intake States:

    public enum ExtendoState {
        EXTENDING,
        RETRACTING,
        REST
    }


    public enum DropdownState {
        TRANSFER,
        INTAKE,
        READY
    }

    public enum GateState {
        CLOSED,
        OPEN_EJECT,
        OPEN_TRANSFER,
        MEGA_CLOSED
    }

    public enum SpinnerState {
        INTAKING,
        REVERSED,
        STOPPED
    }

    //Outtake States:
    public enum OuttakeSlidesState {
        EXTENDING,
        RETRACTING,
        REST
    }

    public enum OuttakeClawState {
        OPEN,
        CLOSED,
        OPEN_TRANSFER
    }

    public enum OuttakeWristState {
        TRANSFER,
        SPECIMEN_GRAB,
        SPECIMEN_DEPOSIT,
        BUCKET_IDEAL
    }

    public enum OuttakeArmState {
        TRANSFER,
        SPECIMEN_GRAB,
        SPECIMEN_DEPOSIT,
        BUCKET_IDEAL
    }

}
