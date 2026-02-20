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

@Autonomous(name = "BLUE 3 largezone")
public class BLUE3largezone extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        MecanumDrive drivebase = robot.drivetrain.getDrive();

        Pose2d initialPose = new Pose2d(53, 48, Math.toRadians(49));
        drivebase.localizer.setPose(initialPose);

        PoseStorage.currentPose = initialPose;
        PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
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
                                        .strafeToLinearHeading(new Vector2d(53, 12), Math.toRadians(70), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-60, 90))
                                        .afterTime(0, new ParallelAction(robot.intake.overrideIntake(true), robot.intake.toggleIntake(true), robot.intake.runIntake()))
                                        .build()
                        )
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