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

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Auton BLUE")
public class C9P9BLUE extends LinearOpMode {
    Robot robot;
    private TeleOpTrajectories trajectories;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true); // Initialize entirety of robot with hardwareMap and telemetry
        MecanumDrive drivebase = robot.drivetrain.getDrive(); // Get the mecanum drivebase for trajectory to work with
        trajectories = TeleOpTrajectories.INSTANCE;

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 20.0); // Optional constraints to change from constraints in MecanumDrive

        Pose2d initialPose = new Pose2d(60, -20, Math.toRadians(180)); // Initialize position and heading of robot. Position is grid of -72, 0, 72 from left to right and bottom to top, heading is 0 - 360, CCW, 0 = north
        drivebase.localizer.setPose(initialPose); // Set the localizer start position to initialPose
        PoseStorage.currentPose = initialPose; // Set global pose value

        List<String> pattern = new ArrayList<>();



        // Autonomous sequence

        // Actions called during init
        Actions.runBlocking(new ParallelAction(robot.intake.toggleIntake()));

        do {
            robot.limelight.setPipeline(Limelight.Pipeline.AT1);
            if (robot.limelight.isDetected()) {
                pattern.addAll(robot.limelight.getPattern());
            }
        } while (pattern.isEmpty());

        waitForStart();

        if (isStopRequested()) return;

        // Main sequence
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                drivebase.actionBuilder(new Pose2d(60, -20, Math.toRadians(180)), false)
                                        .strafeToSplineHeading(new Vector2d(57, -20), Math.toRadians(195))
                                        .stopAndAdd(new SequentialAction(
                                                robot.flywheel.runFlywheel(),
                                                robot.orderedShot(pattern)
                                        ))

                                        .afterTime(0.5, new ParallelAction(
                                                robot.intake.toggleIntake(),
                                                robot.flywheel.stopFlywheel()))
                                        .strafeToSplineHeading(new Vector2d(44.5, -25), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                                new TranslationalVelConstraint(40),
                                                new AngularVelConstraint(Math.PI / 2))))
                                        .splineToConstantHeading(new Vector2d(35.5, -52), Math.toRadians(270), new TranslationalVelConstraint(8))
                                        .afterTime(0, robot.intake.toggleIntake())

                                        .strafeToSplineHeading(new Vector2d(28, -30), Math.toRadians(220))
                                        .splineToConstantHeading(new Vector2d(-5, -15), Math.toRadians(180))
                                        .stopAndAdd(new SequentialAction(
                                                robot.flywheel.runFlywheel(),
                                                robot.orderedShot(pattern)
                                        ))

                                        .afterTime(0.5, new ParallelAction(
                                                robot.intake.toggleIntake(),
                                                robot.flywheel.stopFlywheel()))
                                        .strafeToSplineHeading(new Vector2d(5, -25), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                                new TranslationalVelConstraint(40),
                                                new AngularVelConstraint(Math.PI / 2))))
                                        .splineToConstantHeading(new Vector2d(12, -50), Math.toRadians(270), new TranslationalVelConstraint(10))
                                        .afterTime(0, robot.intake.toggleIntake())

                                        .strafeToSplineHeading(new Vector2d(5, -33), Math.toRadians(220))
                                        .splineToConstantHeading(new Vector2d(-10, -18), Math.toRadians(180))
                                        .stopAndAdd(new SequentialAction(
                                                robot.flywheel.runFlywheel(),
                                                robot.orderedShot(pattern)
                                        ))

                                        .afterTime(0.5, new ParallelAction(
                                                robot.intake.toggleIntake(),
                                                robot.flywheel.stopFlywheel()))
                                        .strafeToSplineHeading(new Vector2d(-10.5, -30), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                                new TranslationalVelConstraint(40),
                                                new AngularVelConstraint(Math.PI / 2))))
                                        .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(270), new TranslationalVelConstraint(9))
                                        .afterTime(0, robot.intake.toggleIntake())

                                        .strafeToSplineHeading(new Vector2d(-29, -24), Math.toRadians(220))
                                        .stopAndAdd(new SequentialAction(
                                                robot.flywheel.runFlywheel(),
                                                robot.orderedShot(pattern)
                                        ))
                                        .strafeTo(new Vector2d(0, -24))
                                        .build()
                        ),
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        robot.flywheel.setVls(trajectories.distanceToTarget(robot.drivetrain, true)),
                        robot.intakePeriodic()
                )
        );

        PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
        PoseStorage.currentPose = drivebase.localizer.getPose(); // Set global pose value for TeleOp to utilize
    }
}
