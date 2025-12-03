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

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.ParentHub;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

@Autonomous(name = "Auton RED")
public class C9P9RED extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        MecanumDrive drivebase = robot.drivetrain.getDrive();

        Pose2d initialPose = new Pose2d(60, -20, Math.toRadians(0));
        drivebase.localizer.setPose(initialPose);
        PoseStorage.currentPose = initialPose;

        // Actions called during init
        Actions.runBlocking(new ParallelAction(robot.intake.toggleIntake(false)));

        waitForStart();

        if (isStopRequested()) return;

        // Main sequence comprised of initial ordered shots, intake first set, shoot second ordered set, intake third set, shoot third ordered set, and park

        // Initial movement to large area and flywheel activation
        Actions.runBlocking(
                new SequentialAction(
                        robot.flywheel.setVls(80),
                        robot.flywheel.runFlywheel(),
                        new RaceAction(
                                new ParallelAction(
                                        robot.spindexer.autonPeriodic(),
                                        robot.flywheel.autonPeriodic(),
                                        robot.limelight.setPattern()
                                ),
                                drivebase.actionBuilder(new Pose2d(60, -20, Math.toRadians(0)), false)
                                        .strafeTo(new Vector2d(40, -20))
                                        .strafeToSplineHeading(new Vector2d(15, -20), Math.toRadians(317), new TranslationalVelConstraint(70))
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
                                robot.flywheel.stopFlywheel()
                        )
                )
        );

        // Intake initial set, move to large launch zone corner, prime flywheel
        Actions.runBlocking(
                new RaceAction(
                        new ParallelAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic()
                        ),
                        robot.intakeAutoPeriodic(),
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                .strafeToSplineHeading(new Vector2d(15, -30), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                        new TranslationalVelConstraint(40),
                                        new AngularVelConstraint(Math.PI / 2))))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(15, -55), Math.toRadians(270), new TranslationalVelConstraint(6))
                                .afterTime(0, new ParallelAction(robot.intake.toggleIntake(false), robot.flywheel.runFlywheel()))

                                .strafeToSplineHeading(new Vector2d(15, -20), Math.toRadians(317))
                                .build()
                )
        );

        // Ordered set of 3 (2)
        if (robot.spindexer.slots.contains(PoseStorage.pattern.get(0))) {
            Actions.runBlocking(
                    new RaceAction(
                            robot.spindexer.autonPeriodic(),
                            robot.flywheel.autonPeriodic(),
                            robot.intakeAutoPeriodic(),
                            new SequentialAction(
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
                                            robot.flywheel.stopFlywheel()
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
                                            robot.spindexer.removeIndexed()
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
                                        robot.spindexer.removeIndexed()
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
                                    robot.spindexer.removeIndexed()
                            )
                    )
            );
        }

        // Intake second set, move to large launch zone, prime flywheel
        Actions.runBlocking(
                new RaceAction(
                        new ParallelAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic()
                        ),
                        robot.intakeAutoPeriodic(),
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                .strafeToSplineHeading(new Vector2d(-4, -33), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                        new TranslationalVelConstraint(40),
                                        new AngularVelConstraint(Math.PI / 2))))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(-4, -55), Math.toRadians(270), new TranslationalVelConstraint(6))
                                .afterTime(0, new ParallelAction(robot.intake.toggleIntake(false), robot.flywheel.runFlywheel()))

                                .strafeToSplineHeading(new Vector2d(15, -20), Math.toRadians(317))
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
                                            robot.flywheel.stopFlywheel()
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
                                            robot.spindexer.removeIndexed()
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
                                        robot.spindexer.removeIndexed()
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
                                    robot.spindexer.removeIndexed()
                            )
                    )
            );
        }

        // Move out of launch zone
        Actions.runBlocking(
                drivebase.actionBuilder(drivebase.localizer.getPose(), true)
                        .strafeToSplineHeading(new Vector2d(-10, -45), 0)
                        .build()
        );

        PoseStorage.currentAlliance = PoseStorage.Alliance.RED;
        PoseStorage.currentPose = drivebase.localizer.getPose(); // Set global pose value for TeleOp to utilize
    }
}