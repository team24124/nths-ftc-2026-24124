package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.Arrays;

@Autonomous(name = "Support Blue")
public class SUPPORTblue extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        MecanumDrive drivebase = robot.drivetrain.getDrive();

        Pose2d initialPose = new Pose2d(-63.3, 15.2, Math.toRadians(0));
        drivebase.localizer.setPose(initialPose);

        PoseStorage.currentPose = initialPose;
        PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
        PoseStorage.pattern.clear();

        robot.spindexer.updateDistance(170);

        Actions.runBlocking(new ParallelAction(robot.intake.overrideIntake(true), robot.intake.toggleIntake(false)));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        robot.flywheel.runFlywheel(),
                        robot.flywheel.setVls(170),
                        new RaceAction(
                                new ParallelAction(
                                        robot.flywheel.autonPeriodic(),
                                        robot.spindexer.autonPeriodic(),
                                        robot.spindexer.sortTo(0)
                                ),
                                drivebase.actionBuilder(initialPose, false)
                                        .strafeToLinearHeading(new Vector2d(-58, 15), Math.toRadians(20))
                                        .waitSeconds(4)
                                        .build()
                        )
                )
        );

        Actions.runBlocking(
                new RaceAction(
                        robot.flywheel.autonPeriodic(),
                        robot.spindexer.autonPeriodic(),
                        new SequentialAction(
                                robot.spindexer.outputTo(0),
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
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-54, 48, Math.toRadians(110)), Math.toRadians(45))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToSplineHeading(new Pose2d(-53, 60, Math.toRadians(150)), Math.toRadians(180))
                                .afterTime(0, robot.flywheel.runFlywheel())
                                .splineToLinearHeading(new Pose2d(-63, 60, Math.toRadians(180)), Math.toRadians(180), new TranslationalVelConstraint(30))
                                .waitSeconds(1)
                                .stopAndAdd(robot.spindexer.sortTo(0))

                                .strafeToSplineHeading(new Vector2d(-58, 15),  Math.toRadians(25))
                                .afterTime(0.2, new ParallelAction(robot.intake.overrideIntake(true), robot.intake.toggleIntake(true), robot.intake.runIntake()))
                                .build()
                )
        );

        // Ordered set of 3
        Actions.runBlocking(
                new RaceAction(
                        robot.flywheel.autonPeriodic(),
                        robot.spindexer.autonPeriodic(),
                        new SequentialAction(
                                new SleepAction(4),
                                robot.spindexer.outputTo(0),
                                robot.spindexer.outputTo(0),
                                robot.spindexer.removeAllIndexed(),
                                new ParallelAction(robot.flywheel.stopFlywheel(), robot.intake.overrideIntake(false), robot.intake.toggleIntake(false), robot.intake.stopIntake())
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
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-54, 25, Math.toRadians(90)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-54, 60), Math.toRadians(280))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(-46, 25), Math.toRadians(90))
                                .afterTime(0, robot.flywheel.runFlywheel())
                                .splineToConstantHeading(new Vector2d(-46, 60), Math.toRadians(90))

                                .strafeToSplineHeading(new Vector2d(-58, 15),  Math.toRadians(25))
                                .afterTime(0.2, new ParallelAction(robot.intake.overrideIntake(true), robot.intake.toggleIntake(true), robot.intake.runIntake(), robot.spindexer.sortTo(0)))
                                .build()
                )
        );

        PoseStorage.currentPose = robot.drivetrain.getPosition(); // Set global pose value for TeleOp to utilize

        // Ordered set of 3
        Actions.runBlocking(
                new RaceAction(
                        robot.flywheel.autonPeriodic(),
                        robot.spindexer.autonPeriodic(),
                        new SequentialAction(
                                new SleepAction(4),
                                robot.spindexer.outputTo(0),
                                robot.spindexer.outputTo(0),
                                robot.spindexer.removeAllIndexed(),
                                new ParallelAction(robot.flywheel.stopFlywheel(), robot.intake.overrideIntake(false), robot.intake.toggleIntake(false), robot.intake.stopIntake())
                        )
                )
        );

        Actions.runBlocking(
                new RaceAction(
                        robot.spindexer.sortTo(3),
                        robot.flywheel.autonPeriodic()
                )
        );
    }
}