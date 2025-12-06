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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.Arrays;

@Autonomous(name = "Auton BLUE 2")
public class C9P9BLUE2 extends LinearOpMode {
    Robot robot;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        MecanumDrive drivebase = robot.drivetrain.getDrive();

        Pose2d initialPose = new Pose2d(-60, 20, Math.toRadians(0));
        drivebase.localizer.setPose(initialPose);
        PoseStorage.currentPose = initialPose;

        // Actions called during init
        Actions.runBlocking(new ParallelAction(robot.intake.toggleIntake(false)));

        robot.spindexer.os.enableOscillation(false);
        PoseStorage.pattern.clear();

        waitForStart();

        do {
            robot.limelight.setPipeline(Limelight.Pipeline.AT1);
            if (robot.limelight.isDetected()) {
                PoseStorage.pattern.addAll(robot.limelight.getPattern());
            }
        } while (PoseStorage.pattern.isEmpty());

        if (isStopRequested()) return;

        // Main sequence comprised of initial ordered shots, intake first set, shoot second ordered set, intake third set, shoot third ordered set, and park

        // Initial movement to large area and flywheel activation
        Actions.runBlocking(
                new SequentialAction(
                        robot.flywheel.setVls(90),
                        robot.flywheel.runFlywheel(),
                        new RaceAction(
                                new ParallelAction(
                                        robot.spindexer.autonPeriodic(),
                                        robot.flywheel.autonPeriodic()
                                ),
                                drivebase.actionBuilder(initialPose, false)
                                        .strafeToSplineHeading(new Vector2d(15, 20), Math.toRadians(42), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-30, 70))
                                        .afterTime(0, new ParallelAction(robot.intake.toggleIntake(true), robot.intake.runIntake()))
                                        .build()
                        )
                )
        );

        // Ordered set of 3
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(PoseStorage.pattern.get(0))),
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
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(PoseStorage.pattern.get(1))),
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
                                robot.spindexer.sortTo(robot.spindexer.slots.indexOf(PoseStorage.pattern.get(2))),
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed(),
                                new ParallelAction(robot.intake.toggleIntake(false), robot.intake.stopIntake()),
                                robot.flywheel.stopFlywheel()
                        )
                )
        );

        // Intake initial set, move to large launch zone corner, prime flywheel
        Actions.runBlocking(
                new RaceAction(
                        new ParallelAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic(),
                                robot.intakeAutoPeriodic()
                        ),
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                .strafeToSplineHeading(new Vector2d(17, 30), Math.toRadians(90), new MinVelConstraint(Arrays.asList(
                                        new TranslationalVelConstraint(60),
                                        new AngularVelConstraint(Math.PI / 2))))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(17, 58), Math.toRadians(270), new TranslationalVelConstraint(8))
                                .afterTime(0, robot.flywheel.runFlywheel())

                                .strafeToSplineHeading(new Vector2d(15, 20), Math.toRadians(44), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-30, 70))
                                .build()
                )
        );

        // Ordered set of 3 (2)
        if (robot.spindexer.slots.contains(PoseStorage.pattern.get(0))) {
            Actions.runBlocking(
                    new RaceAction(
                            robot.spindexer.autonPeriodic(),
                            robot.flywheel.autonPeriodic(),
                            new SequentialAction(
                                    new ParallelAction(robot.intake.toggleIntake(true), robot.intake.runIntake()),
                                    robot.spindexer.sortTo(robot.spindexer.slots.indexOf(PoseStorage.pattern.get(0))),
                                    robot.spindexer.kick(),
                                    robot.spindexer.removeIndexed()
                            )
                    )
            );
            if (robot.spindexer.slots.contains(PoseStorage.pattern.get(1))) {
                Actions.runBlocking(
                        new RaceAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic(),
                                new SequentialAction(
                                        robot.spindexer.sortTo(robot.spindexer.slots.indexOf(PoseStorage.pattern.get(1))),
                                        robot.spindexer.kick(),
                                        robot.spindexer.removeIndexed()
                                )
                        )
                );
                if (robot.spindexer.slots.contains(PoseStorage.pattern.get(2))) {
                    Actions.runBlocking(
                            new RaceAction(
                                    robot.spindexer.autonPeriodic(),
                                    robot.flywheel.autonPeriodic(),
                                    new SequentialAction(
                                            robot.spindexer.sortTo(robot.spindexer.slots.indexOf(PoseStorage.pattern.get(2))),
                                            robot.spindexer.kick(),
                                            robot.spindexer.removeIndexed(),
                                            robot.flywheel.stopFlywheel(),
                                            new ParallelAction(robot.intake.toggleIntake(false), robot.intake.stopIntake())
                                    )
                            )
                    );
                } else {
                    Actions.runBlocking(
                            new RaceAction(
                                    robot.spindexer.autonPeriodic(),
                                    robot.flywheel.autonPeriodic(),
                                    new SequentialAction(
                                            robot.spindexer.sortTo(0),
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
                                            robot.spindexer.sortTo(1),
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
                                            robot.spindexer.sortTo(2),
                                            robot.spindexer.kick(),
                                            robot.spindexer.removeIndexed(),
                                            new ParallelAction(robot.intake.toggleIntake(false), robot.intake.stopIntake()),
                                            robot.flywheel.stopFlywheel()
                                    )
                            )
                    );
                }
            } else {
                Actions.runBlocking(
                        new RaceAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic(),
                                new SequentialAction(
                                        robot.spindexer.sortTo(0),
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
                                        robot.spindexer.sortTo(1),
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
                                        robot.spindexer.sortTo(2),
                                        robot.spindexer.kick(),
                                        robot.spindexer.removeIndexed(),
                                        new ParallelAction(robot.intake.toggleIntake(false), robot.intake.stopIntake()),
                                        robot.flywheel.stopFlywheel()
                                )
                        )
                );
            }
        } else {
            Actions.runBlocking(
                    new RaceAction(
                            robot.spindexer.autonPeriodic(),
                            robot.flywheel.autonPeriodic(),
                            new SequentialAction(
                                    new ParallelAction(robot.intake.toggleIntake(true), robot.intake.runIntake()),
                                    robot.spindexer.sortTo(0),
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
                                    robot.spindexer.sortTo(1),
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
                                    robot.spindexer.sortTo(2),
                                    robot.spindexer.kick(),
                                    robot.spindexer.removeIndexed(),
                                    new ParallelAction(robot.intake.toggleIntake(false), robot.intake.stopIntake()),
                                    robot.flywheel.stopFlywheel()
                            )
                    )
            );
        }

        // Intake second set, move to large launch zone, prime flywheel
        Actions.runBlocking(
                new RaceAction(
                        new ParallelAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic(),
                                robot.intakeAutoPeriodic()
                        ),
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                .strafeToSplineHeading(new Vector2d(-5.5, 33), Math.toRadians(90), new MinVelConstraint(Arrays.asList(
                                        new TranslationalVelConstraint(40),
                                        new AngularVelConstraint(Math.PI / 2))))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(-8, 58), Math.toRadians(270), new TranslationalVelConstraint(7))
                                .afterTime(0, robot.flywheel.runFlywheel())

                                .strafeToSplineHeading(new Vector2d(9, 25), Math.toRadians(71), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-30, 80))
                                .splineToConstantHeading(new Vector2d(35, 20), Math.toRadians(315), new TranslationalVelConstraint(70))
                                .afterTime(0, robot.intake.toggleIntake(false))
                                .build()
                )
        );

        PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
        PoseStorage.currentPose = drivebase.localizer.getPose(); // Set global pose value for TeleOp to utilize

        // Ordered set of 3 (2)
        if (robot.spindexer.slots.contains(PoseStorage.pattern.get(0))) {
            Actions.runBlocking(
                    new RaceAction(
                            robot.spindexer.autonPeriodic(),
                            robot.flywheel.autonPeriodic(),
                            new SequentialAction(
                                    new ParallelAction(robot.intake.toggleIntake(true), robot.intake.runIntake()),
                                    robot.spindexer.sortTo(robot.spindexer.slots.indexOf(PoseStorage.pattern.get(0))),
                                    robot.spindexer.kick(),
                                    robot.spindexer.removeIndexed()
                            )
                    )
            );
        }
        if (robot.spindexer.slots.contains(PoseStorage.pattern.get(1))) {
            Actions.runBlocking(
                    new RaceAction(
                            robot.spindexer.autonPeriodic(),
                            robot.flywheel.autonPeriodic(),
                            new SequentialAction(
                                    robot.spindexer.sortTo(robot.spindexer.slots.indexOf(PoseStorage.pattern.get(1))),
                                    robot.spindexer.kick(),
                                    robot.spindexer.removeIndexed()
                            )
                    )
            );
        }
        if (robot.spindexer.slots.contains(PoseStorage.pattern.get(2))) {
            Actions.runBlocking(
                    new RaceAction(
                            robot.spindexer.autonPeriodic(),
                            robot.flywheel.autonPeriodic(),
                            new SequentialAction(
                                    robot.spindexer.sortTo(robot.spindexer.slots.indexOf(PoseStorage.pattern.get(2))),
                                    robot.spindexer.kick(),
                                    robot.spindexer.removeIndexed(),
                                    robot.flywheel.stopFlywheel(),
                                    new ParallelAction(robot.intake.toggleIntake(false), robot.intake.stopIntake())
                            )
                    )
            );
        }
        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic(),
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.sortTo(3),
                                drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                        .waitSeconds(1)
                                        .build()
                        )
                )
        );
    }
}