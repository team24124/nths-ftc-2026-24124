package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.Arrays;

@Autonomous(name = "Specimen Auto")
public class SpecAuto extends LinearOpMode {
    /**
     * Similar to 24124 in 2025 this robot will be a mecanumdrive robot with 2 viper slides,
     * one arm with one motor,
     * one limelight,
     * a grabber system with two elbow servers, one pivot servo, and one claw servo.
     * additional mechanisms added: flywheel,
     */
    Robot robot;

    // Field error variables
    private final double lateral = 0; // Add to poses where aligning to side walls is necessary
    private final double vertical = 0; // Add to poses where aligning to bottom wall is necessary

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true); // Initialize entirety of robot with hardwaremap and telemetry
        MecanumDrive drivebase = robot.driveTrain.getDrive(); // Get the mecanum drivebase for trajectory to work with

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 20.0); // Optional constraints to change from constraints in mecanumdrive

        Pose2d initialPose = new Pose2d(8, 62 + vertical, Math.toRadians(90)); // Initialize position and heading of robot
        drivebase.localizer.setPose(initialPose); // Set the localizer start position to initialPose
        PoseStorage.currentPose = initialPose; // Set global pose value





        // Autonomous sequence

        TrajectoryActionBuilder first = drivebase.actionBuilder(initialPose, false) // Example of trajectory that does not use extra correction
                .strafeTo(new Vector2d(-18, 31.5), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-10.0, 25.0)) // Aggressive start, lazy deceleration
                .stopAndAdd(robot.arm.moveTo(Arm.State.POSTSCORE));

        Action firstActions = first.build(); // Build trajectory movements (converts TrajectoryActionBuilder to Action type) to be sent into Actions.runBlocking()

        Action secondActions = drivebase.actionBuilder(drivebase.localizer.getPose(), true) // Start trajectory with robot's current Pose2d
                .splineToConstantHeading(new Vector2d(40, 35), Math.toRadians(90), baseVelConstraint, baseAccelConstraint)
                .stopAndAdd(robot.arm.moveTo(Arm.State.GRAB))
                .build(); // Or just convert them immediately

        Actions.runBlocking(new ParallelAction(robot.resetControlArm(), robot.slides.setStateTo(Slides.State.HOME))); // Actions to run during initialization

        waitForStart(); // Wait for start button to be pressed

        if (isStopRequested()) return; // Stop everything smoothly if stop is pressed after initialization

        Actions.runBlocking( // Autonomous action, put commas (,) after actions to add another action to runBlocking()
                new SequentialAction( // Execute actions in order
                        new ParallelAction( // Execute actions in tandem
                                robot.slides.setStateTo(Slides.State.CLIP_HIGH_CHAMBER),
                                robot.arm.moveTo(Arm.State.PRESCORE),
                                firstActions,
                                robot.slides.setStateTo(Slides.State.HOME)
                        ),

                        secondActions, // Second action is executed after first parallel actions
                        drivebase.actionBuilder(drivebase.localizer.getPose(), false) // Actions can be written inside runBlocking
                                .afterTime(0, robot.arm.moveTo(Arm.State.DEFAULT))
                                .splineToConstantHeading(new Vector2d(47, 45 + vertical), 270, new TranslationalVelConstraint(75), new ProfileAccelConstraint(-25, 35)) // Aggressive constraints
                                .stopAndAdd(robot.claw.setClawPosition(Claw.ClawState.OPEN))
                                .build(),

                        new ParallelAction(
                                robot.extendCollection(),
                                drivebase.actionBuilder(drivebase.localizer.getPose(), true) // Enables extra correction for exactly 1 second before stopping every movement. if robot is already in intended position, stops movement
                                        .splineToConstantHeading(new Vector2d(47, 35), Math.toRadians(90))
                                        .build()
                        ),
                        robot.claw.setClawPosition(Claw.ClawState.CLOSED),

                        new ParallelAction(
                                robot.arm.moveTo(Arm.State.DEFAULT),
                                drivebase.actionBuilder(drivebase.localizer.getPose(), false) // Enables extra correction for exactly 1 second before stopping every movement. if robot is already in intended position, stops movement
                                        .splineToConstantHeading(new Vector2d(40, 45 + vertical), Math.toRadians(270), new TranslationalVelConstraint(75), new ProfileAccelConstraint(-25, 35))
                                        .stopAndAdd(robot.claw.setClawPosition(Claw.ClawState.OPEN))
                                        .build()
                        ),

                        // Limelight demo

                        new ParallelAction( // Move to right of submersible
                                robot.resetControlArm(), robot.slides.setStateTo(Slides.State.HOME),
                                drivebase.actionBuilder(drivebase.localizer.getPose(), false)
                                        .splineToConstantHeading(new Vector2d(30, 20), Math.toRadians(0))
                                        .build()
                        ),

                        drivebase.actionBuilder(drivebase.localizer.getPose(), true)
                                .splineToConstantHeading(robot.limelight.PSSTargetPoseFieldSpace(robot.driveTrain.getDrive().localizer.getPose()).position, robot.driveTrain.getHeading())
                                .stopAndAdd(robot.arm.moveTo(Arm.State.GRAB))
                                .build()

                        // Grabs and stops
                )
        );
        PoseStorage.currentPose = drivebase.localizer.getPose();
    }
}
