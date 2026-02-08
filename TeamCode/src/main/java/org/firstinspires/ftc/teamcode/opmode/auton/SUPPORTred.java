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

@Autonomous(name = "Support Red")
public class SUPPORTred extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        MecanumDrive drivebase = robot.drivetrain.getDrive();

        Pose2d initialPose = new Pose2d(-63.3, -15.2, Math.toRadians(0));
        drivebase.localizer.setPose(initialPose);

        PoseStorage.currentPose = initialPose;
        PoseStorage.currentAlliance = PoseStorage.Alliance.RED;
        PoseStorage.pattern.clear();

        Actions.runBlocking(new ParallelAction(robot.intake.overrideIntake(true), robot.intake.toggleIntake(false)));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        robot.flywheel.runFlywheel(),
                        robot.flywheel.setVls(160),
                        new RaceAction(
                                new ParallelAction(
                                        robot.flywheel.autonPeriodic(),
                                        robot.spindexer.autonPeriodic(),
                                        robot.spindexer.sortTo(0)
                                ),
                                drivebase.actionBuilder(initialPose, false)
                                        .strafeToLinearHeading(new Vector2d(-54, -15), Math.toRadians(345))
                                        .waitSeconds(3)
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
                                .strafeToSplineHeading(new Vector2d(-40, -21), Math.toRadians(270), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-70, 150))
                                .afterTime(0, robot.intake.toggleIntake(true))
                                .splineToConstantHeading(new Vector2d(-35.5, -63), Math.toRadians(270), new TranslationalVelConstraint(15))
                                .afterTime(0, new ParallelAction(robot.flywheel.runFlywheel(), robot.spindexer.sortTo(0)))

                                .strafeToSplineHeading(new Vector2d(-54, -15),  Math.toRadians(150))
                                .afterTime(0.2, new ParallelAction(robot.intake.overrideIntake(true), robot.intake.toggleIntake(true), robot.intake.runIntake()))
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
                                new SleepAction(2),
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