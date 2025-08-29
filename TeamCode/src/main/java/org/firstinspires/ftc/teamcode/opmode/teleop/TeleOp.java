package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.DropSpinCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ExtendNoSpinCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.RaiseStopCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.teleop.ReverseCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.teleop.ReverseDownCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.auto.RaisedBucketCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.SpecDepCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.SpecGrabCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.DropResetReadyCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.TransferCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.TransferReadyCmd;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.intake.SpinnerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.outtake.ClawCommand;
import org.firstinspires.ftc.teamcode.common.robot.Globals;
import org.firstinspires.ftc.teamcode.common.robot.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    Gamepad swethaController, ahnafController;
    GamepadEx ahnafButtonController, swethaButtonController;
    public Robot robot;
    public double root = 1;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new Pose2d(0,0,Math.toRadians(0)), true, false);
        robot.extendoSubsystem.extendoSetPosition(0);
        robot.outtakeSlidesSubsystem.outtakeSetPosition(0);
        schedule(new TransferReadyCmd(robot));

        ahnafController = gamepad1;
        swethaController = gamepad2;
        ahnafButtonController = new GamepadEx(gamepad1);
        swethaButtonController = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.extendoSubsystem.extendoSlidesLoop();
        robot.outtakeSlidesSubsystem.outtakeSlidesLoop();

        telemetry.update();

        robot.pinPointDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -0.5 * Math.tan(1.12 * ahnafController.left_stick_y),
                        -0.5 * Math.tan(1.12 * ahnafController.left_stick_x)
                ),
                -0.5 * Math.tan(1.12 * ahnafController.right_stick_x)
        ));

        telemetry.addData("Root: ", root);

        //Ahnaf's Controls:
        if (ahnafController.right_trigger >= .5 && root != 2) {
            root = 2;
            schedule(
                  new DropSpinCmd(robot)
            );
        }

        if (ahnafController.right_trigger < .5 && root!=1) {
            root = 1;
            schedule(
                   new RaiseStopCmd(robot) //raises dropdown, stops spinning
            );
        }
        if (ahnafController.left_trigger > 0 && root != 3){
            root = 3;
            schedule(
                    new ReverseCmd(robot) //raises dropdown, reverses spinner
            );
        }
        if (ahnafController.left_trigger > .8 && root != 4){
            root = 4;
            schedule(
                   new ReverseDownCmd(robot) //puts dropdown in intake pos, reverses spinner
            );
        }
        if (ahnafController.left_trigger == 0 && (root ==4 || root == 3)) {
            root = 5;
            schedule(
                    new SpinnerCommand(robot.spinnerSubsystem, Globals.SpinnerState.STOPPED)
            );
        }

        if (ahnafController.right_bumper) {
            schedule(
                   new TransferCmd(robot)
            );
        }

        if (ahnafController.left_bumper) {
            schedule(new ClawCommand(robot.clawSubsystem, Globals.OuttakeClawState.OPEN));
        }

        if(ahnafController.dpad_up) {
            schedule( new SpecDepCmd(robot));
        }
        if(ahnafController.dpad_down) {
            schedule(new SpecGrabCmd(robot));
        }

        //Swetha's Controls:
        if (swethaController.dpad_up) {
            schedule(
                    new RaisedBucketCmd(robot,Globals.LIFT_HIGH_POS)
            );
        }

        if (swethaController.dpad_down) {
            schedule(
                    new DropResetReadyCmd(robot)
            );
        }
        if (swethaController.square) {
            schedule(
                    new ExtendNoSpinCmd(robot, Globals.EXTENDO_MAX_EXTENSION*1)
            );
        }
        if (swethaController.circle) {
            schedule(
                    new ExtendNoSpinCmd(robot, Globals.EXTENDO_MAX_EXTENSION*0.6)
            );
        }
        if (swethaController.cross) {
            schedule(
                    new ExtendNoSpinCmd(robot, Globals.EXTENDO_MAX_RETRACTION)
            );
        }
    }
}
