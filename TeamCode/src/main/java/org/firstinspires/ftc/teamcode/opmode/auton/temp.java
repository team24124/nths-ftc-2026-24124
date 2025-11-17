package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "tempblue")
public class temp extends LinearOpMode {
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(60, -20, Math.toRadians(0));
        drivetrain = new RobotCentricDrive(hardwareMap, initialPose);
        MecanumDrive drivebase = drivetrain.getDrive();

        drivebase.localizer.setPose(initialPose);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        drivebase.actionBuilder(new Pose2d(60, -20, Math.toRadians(180)), false)
                                .strafeToSplineHeading(new Vector2d(30, -20), Math.toRadians(180))
                                .build()
                )

        );

        PoseStorage.currentPose = drivetrain.getPosition();
        PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
    }
}
