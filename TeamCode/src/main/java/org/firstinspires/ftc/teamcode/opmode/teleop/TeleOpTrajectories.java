package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Utilities;

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

    // Calculates whether to turn right or left to align with the top right corner of the field
    public double rotation(Drivetrain drivebase) {
        double botX = Math.abs(drivebase.getPosition().position.x - 72); // Shifts normal coordinate system so that (0, 0) is at the top right
        double botY = Math.abs(drivebase.getPosition().position.y - 72);

        double theta = Math.atan(botY/botX); // Calculate angle from 90 degrees to target (0 being north)

        if (
                Utilities.isBetween(drivebase.getHeading(), 0, (Math.PI / 2) - theta) // Between 0 and angle to target
                    ||
                Utilities.isBetween(drivebase.getHeading(), (Math.PI / 2) - theta + Math.PI, Math.PI * 2) // Or between opposite angle to target and 360
        ) {
            return 0.8; // Turn right
        } else {
            return -0.8; // Turn left
        }
    }
}
