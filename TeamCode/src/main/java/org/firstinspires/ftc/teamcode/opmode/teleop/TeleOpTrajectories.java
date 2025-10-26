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

    public double theta(Drivetrain drivetrain, double X, double Y) {
        double heading = (drivetrain.getHeading() + Math.PI/2) % (Math.PI*2);
        double botX = drivetrain.getPosition().position.x;
        double botY = drivetrain.getPosition().position.y;

        double theta = Math.atan2(X - botX, -Y + botY); // Similar to (y, x) -> x is vertical, y is lateral +left (reversed to accommodate atan2)
        if (theta < 0) theta += Math.PI*2;

        if ((heading - theta) > Math.PI) {
            return -Math.PI + ((heading - theta) % Math.PI);
        }
        if (Math.abs(heading - theta) > Math.PI) {
            return Math.PI + ((heading - theta) % Math.PI);
        } // Normalizing

        return heading - theta;
    }
}
