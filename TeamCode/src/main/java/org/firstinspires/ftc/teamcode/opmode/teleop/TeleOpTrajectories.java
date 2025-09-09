package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public enum TeleOpTrajectories {
    INSTANCE;

    // Aligns by strafing
    public Action vectorAlign(MecanumDrive drivebase, Pose2d targetPose) {
        return drivebase.actionBuilder(drivebase.localizer.getPose(), true)
                        .splineToConstantHeading(targetPose.position, targetPose.heading.toDouble())
                        .build();
    }

    // Aligns by strafing and turning
    public Action poseAlign(MecanumDrive drivebase, Pose2d targetPose) {
        return drivebase.actionBuilder(drivebase.localizer.getPose(), true)
                        .strafeToSplineHeading(new Vector2d(targetPose.position.x, targetPose.position.y), targetPose.heading.toDouble()) // 0 to face directly into target
                        .build();
    }
}
