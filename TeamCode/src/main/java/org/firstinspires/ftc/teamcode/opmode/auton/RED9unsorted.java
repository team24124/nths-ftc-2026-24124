package org.firstinspires.ftc.teamcode.opmode.auton;

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

@Autonomous(name = "-")
public class RED9unsorted extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        MecanumDrive drivebase = robot.drivetrain.getDrive();

        Pose2d initialPose = new Pose2d(53, -48, Math.toRadians(311));
        drivebase.localizer.setPose(initialPose);

        PoseStorage.currentPose = initialPose;
        PoseStorage.currentAlliance = PoseStorage.Alliance.RED;
        PoseStorage.pattern.clear();

        // Actions called during init
        Actions.runBlocking(new ParallelAction(robot.intake.overrideIntake(true), robot.intake.toggleIntake(false)));

        waitForStart();

        if (isStopRequested()) return;

        // Start flywheel, initial movement shoot 3, intake, open gate start flywheel
        Actions.runBlocking(
                new SequentialAction(
                        robot.flywheel.runFlywheel(),
                        robot.flywheel.setVls(61),
                        new RaceAction(
                                new ParallelAction(
                                        robot.spindexer.autonPeriodic(),
                                        robot.flywheel.autonPeriodic(),
                                        robot.spindexer.sortTo(0),
                                        robot.intakeAutoPeriodic()
                                ),
                                drivebase.actionBuilder(initialPose, false)
                                        .strafeToLinearHeading(new Vector2d(27, -19), Math.toRadians(313), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-60, 90))
                                        .build()
                        )
                )
        );

        Actions.runBlocking(
                new RaceAction(
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.outputTo(0),
                                robot.spindexer.removeAllIndexed(),
                                new ParallelAction(robot.flywheel.stopFlywheel(), robot.intake.overrideIntake(false), robot.intake.toggleIntake(true), robot.intake.runIntake())
                        )
                )
        );

        Actions.runBlocking(
                new RaceAction(
                        new ParallelAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic(),
                                robot.intakeAutoPeriodic()
                        ),
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(15, -24, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-50, 130))
                                .afterTime(0, new ParallelAction(robot.intake.overrideIntake(false)))
                                .splineToConstantHeading(new Vector2d(13, -53), Math.toRadians(90), new TranslationalVelConstraint(11))
                                .afterTime(0.3, new ParallelAction(robot.flywheel.runFlywheel(), robot.intake.toggleIntake(false)))
                                .splineToConstantHeading(new Vector2d(8, -52), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(3, -58), Math.toRadians(270))
                                .build()
                )
        );

        // Move to shoot
        Actions.runBlocking(
                new RaceAction(
                        new ParallelAction(
                                robot.spindexer.autonPeriodic(),
                                robot.flywheel.autonPeriodic()
                        ),
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                .strafeToSplineHeading(new Vector2d(25, -16), Math.toRadians(321))
                                .afterTime(0, new ParallelAction(robot.spindexer.sortTo(0), robot.intake.overrideIntake(true), robot.intake.toggleIntake(true), robot.intake.runIntake()))
                                .build()
                )
        );

        // Ordered set of 3
        Actions.runBlocking(
                new RaceAction(
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.outputTo(0),
                                robot.spindexer.removeAllIndexed(),
                                new ParallelAction(robot.flywheel.stopFlywheel(), robot.intake.overrideIntake(false), robot.intake.toggleIntake(false), robot.intake.stopIntake())
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
                                .strafeToSplineHeading(new Vector2d(-5, -25), Math.toRadians(270), new TranslationalVelConstraint(160), new ProfileAccelConstraint(-70, 200))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(-9, -60.5), Math.toRadians(75), new TranslationalVelConstraint(15))
                                .afterTime(0, robot.flywheel.runFlywheel())

                                .splineToSplineHeading(new Pose2d(-5, -33, Math.toRadians(330)), Math.toRadians(58), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-90, 90))
                                .afterTime(0.3, new ParallelAction(robot.intake.toggleIntake(false), robot.spindexer.sortTo(0)))
                                .splineToConstantHeading(new Vector2d(25, -16), Math.toRadians(0), new TranslationalVelConstraint(130), new ProfileAccelConstraint(-60, 130))
                                .afterTime(0, new ParallelAction(robot.intake.overrideIntake(true), robot.intake.toggleIntake(true), robot.intake.runIntake()))
                                .build()
                )
        );

        // Ordered set of 3
        Actions.runBlocking(
                new RaceAction(
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.outputTo(0),
                                robot.spindexer.outputTo(0),
                                robot.spindexer.removeAllIndexed(),
                                new ParallelAction(robot.flywheel.stopFlywheel(), robot.intake.overrideIntake(false), robot.intake.toggleIntake(false), robot.intake.stopIntake())
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
                                .strafeToSplineHeading(new Vector2d(-25.5, -24), Math.toRadians(270), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-70, 400))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(-35.5, -63), Math.toRadians(270), new TranslationalVelConstraint(15))
                                .afterTime(0, new ParallelAction(robot.flywheel.runFlywheel(), robot.spindexer.sortTo(0)))

                                .strafeToSplineHeading(new Vector2d(36, -15),  Math.toRadians(305), new TranslationalVelConstraint(120))
                                .afterTime(0.5, new ParallelAction(robot.intake.overrideIntake(true), robot.intake.toggleIntake(true), robot.intake.runIntake()))
                                .build()
                )
        );

        PoseStorage.currentPose = robot.drivetrain.getPosition(); // Set global pose value for TeleOp to utilize

        // Ordered set of 3
        Actions.runBlocking(
                new RaceAction(
                        robot.flywheel.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.outputTo(0),
                                robot.spindexer.outputTo(0),
                                robot.spindexer.removeAllIndexed(),
                                new ParallelAction(robot.flywheel.stopFlywheel(), robot.intake.overrideIntake(false), robot.intake.toggleIntake(false), robot.intake.stopIntake())
                        )
                )
        );

        robot.spindexer.states.setSelected(3);

        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.autonPeriodic()
                )
        );
    }
}