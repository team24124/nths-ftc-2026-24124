package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

import java.util.List;

@TeleOp(name = "BlueF", group = "!")
public class BlueF extends OpMode {
    private Robot robot;
    private GamepadEx driver, operator;
    private TeleOpTrajectories trajectories;
    private List<LynxModule> hubs;
    private boolean alignToAT = false;
    private boolean trajectoryAlign = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot = new Robot(hardwareMap, telemetry, true);
        robot.actions = ActionScheduler.INSTANCE;
        trajectories = TeleOpTrajectories.INSTANCE;
        robot.actions.init();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // --- Driver inputs ---
        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.drivetrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.drivetrain.getSpeeds()::next));
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.limelight.setPipeline(Limelight.Pipeline.AT2);
            trajectoryAlign = false;
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            robot.limelight.setPipeline(Limelight.Pipeline.AT4);
            trajectoryAlign = false;
            alignToAT = false;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            robot.limelight.setPipeline(Limelight.Pipeline.AT2);
            alignToAT = false;
            trajectoryAlign = true;
        }

        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            if (robot.limelight.isDetected()) {
                robot.actions.schedule(trajectories.poseAlign(robot.drivetrain.getDrive(), robot.limelight.ATTargetPoseFieldSpace(robot.drivetrain.getDrive().localizer.getPose())));
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
            if (robot.limelight.isDetected() && !alignToAT && !trajectoryAlign) {
                robot.drivetrain.getDrive().localizer.setPose(robot.limelight.ATRobotPoseFieldSpace());
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            Vector2d current = robot.drivetrain.getDrive().localizer.getPose().position;
            robot.drivetrain.getDrive().localizer.setPose(new Pose2d(current, 0));
        }

        // --- Operator inputs ---
        if (operator.isDown(GamepadKeys.Button.A)) {
            robot.actions.schedule(new InstantAction(() ->
                    robot.flyWheel.runFlyWheel(robot.limelight.distance())
            ));
        } else if (robot.flyWheel.powered) {
            robot.actions.schedule(new InstantAction(() ->
                    robot.flyWheel.stopFlyWheel()
            ));
        }

        // --- Periodic calls ---
        if (!robot.drivetrain.getDrive().isBusy) {
            if (alignToAT) {
                if (robot.limelight.isDetected()) {
                    robot.drivetrain.drive(x, y, robot.limelight.degreeOffset(), true);
                } else {
                    robot.drivetrain.drive(x, y, trajectories.rotation(robot.drivetrain, -72), false);
                }
            } else {
                robot.drivetrain.drive(x, y, rx, false);
            }
        }
        robot.drivetrain.periodic();

        driver.readButtons();
        operator.readButtons();

        robot.turretBase.setHeadings(robot.drivetrain.getDrive(), robot.limelight.degreeOffset(), robot.limelight.isDetected(), false);
        robot.turretBase.periodic();

        robot.telemetryControl.update();
        robot.actions.run();
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}