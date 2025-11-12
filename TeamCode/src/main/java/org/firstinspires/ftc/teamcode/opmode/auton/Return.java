package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.Arrays;

@Autonomous(name = "Return")
public class Return extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true); // Initialize entirety of robot with hardwareMap and telemetry
        MecanumDrive drivebase = robot.drivetrain.getDrive(); // Get the mecanum drivebase for trajectory to work with

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(65.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 20.0); // Optional constraints to change from constraints in MecanumDrive

        Pose2d initialPose = PoseStorage.currentPose; // Initialize position and heading of robot to global pose
        drivebase.localizer.setPose(initialPose); // Set the localizer start position to initialPose
        double endY = (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) ? -20 : 20; // Set ending pose

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drivebase.actionBuilder(PoseStorage.currentPose, true)
                        .strafeToSplineHeading(new Vector2d(60, endY), Math.toRadians(180))
                        .afterTime(0, robot.spindexer.intakeToEmpty())
                        .build()
        );

        PoseStorage.currentPose = drivebase.localizer.getPose(); // Set global pose value for TeleOp to utilize
    }
}
