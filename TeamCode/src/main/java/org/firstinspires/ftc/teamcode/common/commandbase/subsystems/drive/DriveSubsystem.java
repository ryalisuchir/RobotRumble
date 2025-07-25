package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.roadrunner.PinpointDrive;

public class DriveSubsystem extends SubsystemBase { //done 07.20.2025
    private final PinpointDrive drive;
    private final boolean fieldCentric;

    public DriveSubsystem(PinpointDrive drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public Pose2d getPoseEstimate() {
        return new Pose2d(drive.pose.position, drive.pose.heading);
    }

    public void setPoseEstimate(Pose2d initialPose) {
        drive.pose = initialPose;
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public TrajectoryActionBuilder trajectoryActionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }

    public TrajectoryActionBuilder trajectoryActionBuilderCorrection(Pose2d startPose) {
        return drive.actionBuilderAddedCorrection(startPose);
    }

}
