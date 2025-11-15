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

@Autonomous(name = "temp")
public class temp extends LinearOpMode {

    private Drivetrain drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new RobotCentricDrive(hardwareMap, new Pose2d(0, 0, 0)); // Initialize entirety of robot with hardwareMap and telemetry
        MecanumDrive drivebase = drivetrain.getDrive(); // Get the mecanum drivebase for trajectory to work with

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 20.0); // Optional constraints to change from constraints in MecanumDrive

        Pose2d initialPose = new Pose2d(60, -20, Math.toRadians(180)); // Initialize position and heading of robot. Position is grid of -72, 0, 72 from left to right and bottom to top, heading is 0 - 360, CCW, 0 = north
        drivebase.localizer.setPose(initialPose); // Set the localizer start position to initialPose
        PoseStorage.currentPose = initialPose; // Set global pose value





        // Autonomous sequence
        waitForStart();

        if (isStopRequested()) return;

        // Main sequence
        Actions.runBlocking(
                new SequentialAction(
                        drivebase.actionBuilder(new Pose2d(60, -20, Math.toRadians(180)), false)
                                .strafeToSplineHeading(new Vector2d(30, -20), Math.toRadians(270))
                                .build()
                )

        );

        PoseStorage.currentPose = new Pose2d(0, 0, 0); // Set global pose value for TeleOp to utilize
    }
}
