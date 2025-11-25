package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
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

@Autonomous(name = "Auton RED")
public class C9P9RED extends LinearOpMode {
    Robot robot;
    private TeleOpTrajectories trajectories;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        MecanumDrive drivebase = robot.drivetrain.getDrive();
        trajectories = TeleOpTrajectories.INSTANCE;

        Pose2d initialPose = new Pose2d(-60, -20, Math.toRadians(0));
        drivebase.localizer.setPose(initialPose);
        PoseStorage.currentPose = initialPose;

        List<String> pattern = new ArrayList<>();

        // Actions called during init
        Actions.runBlocking(new ParallelAction(robot.intake.toggleIntake(false)));

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
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        robot.intakeAutoPeriodic(),
                        new SequentialAction(
                                drivebase.actionBuilder(new Pose2d(-60, -20, Math.toRadians(0)), false)
                                        .strafeToSplineHeading(new Vector2d(-57, -20), Math.toRadians(345))
                                        .stopAndAdd(new SequentialAction(
                                                robot.flywheel.setVls(trajectories.distanceToTarget(robot.drivetrain, false)),
                                                robot.flywheel.runFlywheel(),
                                                robot.orderedShot(pattern)
                                        ))

                                        .afterTime(0.5, new ParallelAction(
                                                robot.intake.toggleIntake(true),
                                                robot.flywheel.stopFlywheel()))
                                        .strafeToSplineHeading(new Vector2d(-36, -30), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                                new TranslationalVelConstraint(40),
                                                new AngularVelConstraint(Math.PI / 2))))
                                        .splineToConstantHeading(new Vector2d(-35, -61), Math.toRadians(270), new TranslationalVelConstraint(5.5))
                                        .afterTime(0, robot.intake.toggleIntake(false))

                                        .strafeToSplineHeading(new Vector2d(-28, -30), Math.toRadians(320))
                                        .splineToConstantHeading(new Vector2d(5, -15), Math.toRadians(0))
                                        .stopAndAdd(new SequentialAction(
                                                robot.flywheel.setVls(trajectories.distanceToTarget(robot.drivetrain, false)),
                                                //robot.flywheel.runFlywheel(),
                                                robot.orderedShot(pattern)
                                        ))

                                        .afterTime(0.5, new ParallelAction(
                                                robot.intake.toggleIntake(true),
                                                robot.flywheel.stopFlywheel()))
                                        .strafeToSplineHeading(new Vector2d(-5, -25), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                                new TranslationalVelConstraint(40),
                                                new AngularVelConstraint(Math.PI / 2))))
                                        .splineToConstantHeading(new Vector2d(-12, -59), Math.toRadians(270), new TranslationalVelConstraint(5.5))
                                        .afterTime(0, robot.intake.toggleIntake(false))

                                        .strafeToSplineHeading(new Vector2d(-5, -33), Math.toRadians(320))
                                        .splineToConstantHeading(new Vector2d(10, -18), Math.toRadians(0))
                                        .stopAndAdd(new SequentialAction(
                                                robot.flywheel.setVls(trajectories.distanceToTarget(robot.drivetrain, false)),
                                                //robot.flywheel.runFlywheel(),
                                                robot.orderedShot(pattern)
                                        ))

                                        .afterTime(0.5, new ParallelAction(
                                                robot.intake.toggleIntake(true),
                                                robot.flywheel.stopFlywheel()))
                                        .strafeToSplineHeading(new Vector2d(10.5, -30), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                                new TranslationalVelConstraint(40),
                                                new AngularVelConstraint(Math.PI / 2))))
                                        .splineToConstantHeading(new Vector2d(12, -48), Math.toRadians(270), new TranslationalVelConstraint(5.5))
                                        .afterTime(0, robot.intake.toggleIntake(false))

                                        .strafeToSplineHeading(new Vector2d(29, -24), Math.toRadians(320))
                                        .stopAndAdd(new SequentialAction(
                                                //robot.flywheel.runFlywheel(),
                                                robot.orderedShot(pattern)
                                        ))
                                        .strafeTo(new Vector2d(0, -24))
                                        .build()

                        )
                )
        );

        PoseStorage.currentAlliance = PoseStorage.Alliance.RED;
        PoseStorage.currentPose = drivebase.localizer.getPose(); // Set global pose value for TeleOp to utilize
    }
}
