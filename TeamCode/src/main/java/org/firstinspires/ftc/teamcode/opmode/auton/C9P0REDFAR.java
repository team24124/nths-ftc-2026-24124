package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Auton RED2")
public class C9P0REDFAR extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        MecanumDrive drivebase = robot.drivetrain.getDrive();

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

        // Main sequence comprised of initial shots, intake first set, shoot first ordered set, intake second set, shoot second ordered set, intake third set, shoot third ordered set, and park

        // Initial nudge and flywheel activation
        Actions.runBlocking(
                new SequentialAction(
                        robot.flywheel.setVls(154),
                        robot.flywheel.runFlywheel(),
                        new RaceAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic(),
                                drivebase.actionBuilder(new Pose2d(-60, -20, Math.toRadians(0)), false)
                                        .strafeToSplineHeading(new Vector2d(-57, -20), Math.toRadians(345))
                                        .build()
                        )
                )
        );

        // Unordered set of 3
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(0))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed()
                        )
                )
        );
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(1))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed()
                        )
                )
        );
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(2))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed(),
                                robot.flywheel.stopFlywheel()
                        )
                )
        );

        // Intake initial set, move to large small zone, prime flywheel
        Actions.runBlocking(
                new RaceAction(
                        new ParallelAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic(),
                                robot.intakeAutoPeriodic()
                        ),
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                .strafeToSplineHeading(new Vector2d(-36, -30), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                        new TranslationalVelConstraint(40),
                                        new AngularVelConstraint(Math.PI / 2))))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(-35, -52), Math.toRadians(270), new TranslationalVelConstraint(2.4))
                                .afterTime(0, robot.intake.toggleIntake(false))

                                .strafeToSplineHeading(new Vector2d(-28, -30), Math.toRadians(312))
                                .splineToConstantHeading(new Vector2d(10, -10), Math.toRadians(0))
                                .afterTime(0, new ParallelAction(
                                        robot.flywheel.setVls(94),
                                        robot.flywheel.runFlywheel()))
                                .build()
                )
        );

        // Ordered set of 3 (1)
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(0))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed()
                        )
                )
        );
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(1))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed()
                        )
                )
        );
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(2))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed(),
                                robot.flywheel.stopFlywheel()
                        )
                )
        );

        // Intake second set, move to large launch zone, prime flywheel
        Actions.runBlocking(
                new RaceAction(
                        new ParallelAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic(),
                                robot.intakeAutoPeriodic()
                        ),
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                .strafeToSplineHeading(new Vector2d(-5, -25), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                        new TranslationalVelConstraint(40),
                                        new AngularVelConstraint(Math.PI / 2))))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(-12, -59), Math.toRadians(270), new TranslationalVelConstraint(2.6))
                                .afterTime(0, robot.intake.toggleIntake(false))

                                .strafeToSplineHeading(new Vector2d(-5, -33), Math.toRadians(320))
                                .splineToConstantHeading(new Vector2d(10, -18), Math.toRadians(0))
                                .afterTime(0, robot.flywheel.runFlywheel())
                                .build()
                )
        );

        // Ordered set of 3 (2)
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(0))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed()
                        )
                )
        );
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(1))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed()
                        )
                )
        );
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(2))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed(),
                                robot.flywheel.stopFlywheel()
                        )
                )
        );

        // Intake third set, move to large launch zone, prime flywheel
        Actions.runBlocking(
                new RaceAction(
                        new ParallelAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic(),
                                robot.intakeAutoPeriodic()
                        ),
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                .strafeToSplineHeading(new Vector2d(10.5, -30), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                        new TranslationalVelConstraint(40),
                                        new AngularVelConstraint(Math.PI / 2))))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(12, -48), Math.toRadians(270), new TranslationalVelConstraint(2.6))
                                .afterTime(0, robot.intake.toggleIntake(false))

                                .strafeToSplineHeading(new Vector2d(29, -24), Math.toRadians(320))
                                .afterTime(0, robot.flywheel.runFlywheel())
                                .build()
                )
        );

        // Ordered set of 3 (3)
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(0))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed()
                        )
                )
        );
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(1))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed()
                        )
                )
        );
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(pattern.get(2))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed(),
                                robot.flywheel.stopFlywheel()
                        )
                )
        );

        // Move out of launch zone
        Actions.runBlocking(
                drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                        .strafeTo(new Vector2d(0, -24))
                        .build()
        );

        PoseStorage.currentAlliance = PoseStorage.Alliance.RED;
        PoseStorage.currentPose = drivebase.localizer.getPose(); // Set global pose value for TeleOp to utilize
    }
}