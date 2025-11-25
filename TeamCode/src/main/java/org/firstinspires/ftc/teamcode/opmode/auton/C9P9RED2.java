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
import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Auton RED2")
public class C9P9RED2 extends LinearOpMode {
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

        robot.spindexer.states.setSelected(Spindexer.State.SLOT1);
        Actions.runBlocking(
                robot.spindexer.moveToState()
        );
        robot.spindexer.states.setSelected(Spindexer.State.SLOT2);
        Actions.runBlocking(
                robot.spindexer.moveToState()
        );
        robot.spindexer.states.setSelected(Spindexer.State.SLOT3);
        Actions.runBlocking(
                robot.spindexer.moveToState()
        );

        PoseStorage.currentAlliance = PoseStorage.Alliance.RED;
        PoseStorage.currentPose = drivebase.localizer.getPose();
    }
}
