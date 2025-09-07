package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public enum TeleOpTrajectories {
    INSTANCE;

    // Aligns by strafing
    public Action vectorAlign(MecanumDrive drivebase, Vector2d targetPose) {
        Pose2d temporary = new Pose2d(0,0,drivebase.localizer.getPose().heading.toDouble());
        drivebase.localizer.setPose(temporary);
        return drivebase.actionBuilder(temporary, true)
                        .splineToConstantHeading(targetPose, drivebase.localizer.getPose().heading.toDouble())
                        .build();
    }

    // Aligns by strafing and turning
    public Action poseAlign(MecanumDrive drivebase, Pose2d targetPose) {
        Pose2d temporary = new Pose2d(0,0,targetPose.heading.toDouble());
        drivebase.localizer.setPose(temporary);
        return drivebase.actionBuilder(temporary, true)
                        .strafeToSplineHeading(new Vector2d(targetPose.position.x, targetPose.position.y), 0) // 0 to face directly into target
                        .build();
    }

    // Aligns by strafing and turning
    public Action FieldAlignWithMT2(MecanumDrive drivebase, Pose2d botPose) {
        drivebase.localizer.setPose(botPose);
        return drivebase.actionBuilder(botPose, true)
                        .strafeToSplineHeading(new Vector2d(900, 900), 900) // Preset location
                        .build();
    }
}
