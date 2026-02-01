package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.Arrays;

@Autonomous(name = "Idle Blue")
public class IDLEblue extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        MecanumDrive drivebase = robot.drivetrain.getDrive();

        Pose2d initialPose = new Pose2d(-63, 16, Math.toRadians(0));
        drivebase.localizer.setPose(initialPose);

        PoseStorage.currentPose = initialPose;
        PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
        PoseStorage.pattern.clear();

        Actions.runBlocking(new ParallelAction(robot.intake.toggleIntake(false)));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drivebase.actionBuilder(drivebase.localizer.getPose(), true)
                        .strafeToSplineHeading(new Vector2d(-55, 40), Math.toRadians(315))
                        .build()
        );

        PoseStorage.currentPose = drivebase.localizer.getPose();
    }
}