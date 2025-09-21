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

    // Calculates whether to turn right or left to align with the corners of the field
    public double rotation(Drivetrain drivebase, double X) {
        double heading = (drivebase.getHeading() + Math.PI/2) % (Math.PI*2);
        double botX = drivebase.getPosition().position.x;
        double botY = drivebase.getPosition().position.y;

        double theta = Math.atan2(72 - botY, X - botX); // Calculate angle to target (0 being east CCW)
        if (theta < 0) theta += Math.PI*2; // Normalize to 0 - 360

        if (Utilities.isBetween(heading, theta - Math.toRadians(15), theta + Math.toRadians(15))) { // Potential flickers may cause unnecessary rotation, a threshold is necessary
            return 0;
        }
        if (
                Utilities.isBetween(heading, theta, Math.PI + theta) // Between angle to target from 0 = east & opposite
        ) {
            return 0.8; // Turn right
        } else {
            return -0.8; // Turn left
        }
    }
}
