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
    public Action vectorAlign(MecanumDrive drivetrain, Vector2d targetPose) {
        return drivetrain.actionBuilder(drivetrain.localizer.getPose(), true)
                        .strafeTo(targetPose)
                        .build();
    }

    // Aligns by strafing and turning
    public Action poseAlign(MecanumDrive drivetrain, Pose2d targetPose) {
        return drivetrain.actionBuilder(drivetrain.localizer.getPose(), true)
                        .strafeToSplineHeading(new Vector2d(targetPose.position.x, targetPose.position.y), targetPose.heading.toDouble()) // 0 to face directly into target
                        .build();
    }

    // Calculates whether to turn right or left to align with the corners of the field
    public double rotation(Drivetrain drivetrain, double X) {
        double heading = (drivetrain.getHeading() + Math.PI/2) % (Math.PI*2);
        double botX = drivetrain.getPosition().position.x;
        double botY = drivetrain.getPosition().position.y;

        double theta = Math.atan2(72 - botY, X - botX); // Calculate angle to target (0 being east CCW)
        if (theta < 0) theta += Math.PI*2; // Normalize to 0 - 360

        if (Utilities.isBetween(heading, theta - Math.toRadians(15), theta + Math.toRadians(15))) { // Potential flickers may cause unnecessary rotation, a threshold is necessary
            return 0;
        } else if (
                Utilities.isBetween(heading, theta, Math.PI + theta) // Between angle to target from 0 = east & opposite
        ) {
            return 0.8; // Turn right
        } else {
            return -0.8; // Turn left
        }
    }

    public double theta(Drivetrain drivetrain, double X) { // TODO FIX COORDINATE SYSTEM
        double heading = (drivetrain.getHeading() + Math.PI/2) % (Math.PI*2);
        double botX = drivetrain.getPosition().position.x;
        double botY = drivetrain.getPosition().position.y;

        double theta = Math.atan2(72 - botY, X - botX);
        if (theta < 0) theta += Math.PI*2;

        return heading - theta;
    }
}
