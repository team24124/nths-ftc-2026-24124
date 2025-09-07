package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.Arrays;

@Autonomous(name = "Specimen Auto")
public class SpecAuto extends LinearOpMode {
    /**
     * Similar to 24124 in 2025 this robot will be a mecanumdrive robot with 2 viper slides,
     * one arm with one motor,
     * one limelight,
     * a grabber system with two elbow servers, one pivot servo, and one claw servo.
     * additional mechanisms added: flywheel,
     */
    Robot robot;

    // Field error variables
    private final double lateral = 0; // Add to poses where aligning to side walls is necessary
    private final double vertical = 0; // Add to poses where aligning to bottom wall is necessary

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true); // Initialize entirety of robot with hardwaremap and telemetry
        MecanumDrive drivebase = robot.driveTrain.getDrive(); // Get the mecanum drivebase for trajectory to work with

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 20.0); // Optional constraints to change from constraints in mecanumdrive

        Pose2d initialPose = new Pose2d(8, 62 + vertical, Math.toRadians(90)); // Initialize position and heading of robot
        drivebase.localizer.setPose(initialPose); // Set the localizer start position to initialPose
        PoseStorage.currentPose = initialPose; // Set global pose value





        // Autonomous sequence

        Actions.runBlocking(new ParallelAction());

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(

                )
        );
        PoseStorage.currentPose = drivebase.localizer.getPose();
    }
}
