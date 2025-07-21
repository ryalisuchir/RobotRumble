package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public static double extendoStaticMax = 15;
    public static Globals.ExtendoFailState extendoFailState = ExtendoFailState.GOOD;
    public static Globals.ExtendoState extendoState = ExtendoState.REST;
    public static Globals.DropdownState dropdownState = DropdownState.TRANSFER;
    public static Globals.GateState gateState = GateState.CLOSED;
    public static Globals.SpinnerState spinnerState = SpinnerState.STOPPED;
    public static Globals.OuttakeSlidesState outtakeSlidesState = OuttakeSlidesState.REST;
    public static Globals.OuttakeClawState outtakeClawState = OuttakeClawState.CLOSED;
    public static Globals.OuttakeWristState outtakeWristState = OuttakeWristState.TRANSFER;
    public static Globals.OuttakeArmState outtakeArmState = OuttakeArmState.SPECIMEN_GRAB;

    // Lift Subsystem Constants
    public static double LIFT_HIGH_POS = 0;
    public static double LIFT_MID_POS = 0;
    public static double LIFT_PARK_POS = 0;
    public static double LIFT_SPECIMEN_0_POS = 0;
    public static double LIFT_SPECIMEN_1_POS = 0;
    public static double LIFT_RETRACT_POS = -5;
    public static double LIFT_MAX_TOLERANCE = 25;

    //Extendo Subsystem Constants
    public static double EXTENDO_MAX_EXTENSION = 0;
    public static double EXTENDO_MAX_RETRACTION = -5;
    public static double EXTENDO_MAX_TOLERANCE = 25;

    //Dropdown Subsystem Constants
    public static double DROPDOWN_TRANSFER = 0;
    public static double DROPDOWN_GROUND = 0;
    public static double DROPDOWN_HALF = 0;

    //Gate Subsystem Constants
    public static double GATE_CLOSED = 0;
    public static double GATE_OPEN = 0;

    //Outtake Claw Constants
    public static double OUTTAKE_CLAW_OPEN;
    public static double OUTTAKE_CLAW_TRANSFER_OPEN;
    public static double OUTTAKE_CLAW_CLOSE;

    //Outtake Wrist Constants
    public static double OUTTAKE_WRIST_SPECIMEN_GRAB = 0;
    public static double OUTTAKE_WRIST_SPECIMEN_DEPOSIT = 0;
    public static double OUTTAKE_WRIST_TRANSFER = 0;

    //Outtake Arm Constants:
    public static double OUTTAKE_ARM_TRANSFER = 0;
    public static double OUTTAKE_ARM_SPECIMEN_GRAB = 0;
    public static double OUTTAKE_ARM_SPECIMEN_DEPOSIT = 0;
    public static double OUTTAKE_ARM_SPECIMEN_BUCKET_IDEAL = 0;

    //Intake States:
    public enum ExtendoFailState { // This will be used to track current on extendo motors.
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }

    public enum ExtendoState {
        EXTENDING,
        RETRACTING,
        REST
    }

    public enum DropdownState {
        TRANSFER,
        GROUND,
        HALF
    }

    public enum GateState {
        CLOSED,
        OPEN
    }

    public enum SpinnerState {
        INTAKING,
        EJECTING,
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
        SPECIMEN_DEPOSIT
    }

    public enum OuttakeArmState {
        TRANSFER,
        SPECIMEN_GRAB,
        SPECIMEN_DEPOSIT,
        BUCKET_IDEAL
    }

}
