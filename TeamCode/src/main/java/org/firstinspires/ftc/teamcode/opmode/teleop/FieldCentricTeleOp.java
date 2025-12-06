package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

@TeleOp(name = "Field Centric TeleOp", group = "!")
public class FieldCentricTeleOp extends OpMode {
    private Robot robot;
    private GamepadEx driver, operator;
    private TeleOpTrajectories trajectories;
    private List<LynxModule> hubs;
    private boolean alignToAT = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        trajectories = TeleOpTrajectories.INSTANCE;
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry, false);
        robot.actions.init();

        // Startup actions
        if (!robot.intake.toggled) {
            robot.intake.toggled = true;
        }
        robot.spindexer.slots = new ArrayList<>(Arrays.asList("empty", "empty", "empty"));
        if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
            robot.limelight.setPipeline(Limelight.Pipeline.AT3);
        } else {
            robot.limelight.setPipeline(Limelight.Pipeline.AT2);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // --------- Driver inputs ---------
        double x = -driver.getLeftY();
        double y = driver.getLeftX();
        if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
            x = driver.getLeftY();
            y = -driver.getLeftX();
        }
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            robot.drivetrain.toggleSpeeds();
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            alignToAT = !alignToAT;
        }

//        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
//            Pose2d targetPose;
//            alignToAT = false;
//            if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
//                targetPose = new Pose2d(-40, -30, Math.toRadians(180));
//            } else {
//                targetPose = new Pose2d(-40, 30, Math.toRadians(180));
//            }
//            robot.actions.schedule(trajectories.poseAlign(robot.drivetrain.getDrive(), targetPose));
//        }

        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            Vector2d current = robot.drivetrain.getDrive().localizer.getPose().position;
            robot.drivetrain.getDrive().localizer.setPose(new Pose2d(current, 0));
        }

        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.spindexer.os.enableOscillation(false);
            robot.actions.schedule(robot.intakePeriodic());
        } else if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER) && robot.spindexer.states.getSelectedIndex() > 2) {
            robot.spindexer.os.enableOscillation(false);
            robot.actions.schedule(new ParallelAction(robot.intake.reverseIntake(), robot.spindexer.removeIndexed(robot.spindexer.states.getSelectedIndex() - 3)));
        } else {
            robot.spindexer.os.enableOscillation(true);
            robot.actions.schedule(robot.intake.stopIntake());
        }

        // --------- Operator inputs ---------
        if (operator.isDown(GamepadKeys.Button.A)) {
            robot.actions.schedule(robot.flywheel.runFlywheel());
        } else {
            robot.actions.schedule(robot.flywheel.stopFlywheel());
        }

        if (robot.flywheel.primed) {
            if (operator.isDown(GamepadKeys.Button.X)) {
                robot.actions.schedule(robot.spindexer.shootOne());
            }
            if (operator.wasJustPressed(GamepadKeys.Button.Y) && robot.spindexer.states.getSelectedIndex() < 3) {
                robot.actions.schedule(
                        new SequentialAction(
                                robot.spindexer.kick(),
                                robot.spindexer.removeIndexed()
                        )
                );
            }
            if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && robot.spindexer.slots.contains("purple")) {
                robot.actions.schedule(robot.shootColor("purple"));
            }
            if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && robot.spindexer.slots.contains("green")) {
                robot.actions.schedule(robot.shootColor("green"));
            }
        }

        // --- Periodic calls ---
        driver.readButtons();
        operator.readButtons();

        if (!robot.drivetrain.getDrive().isBusy) {
            if (alignToAT) {
                if (robot.limelight.isDetected()) {
                    robot.drivetrain.drive(x, y, Math.toRadians(robot.limelight.degreeOffset()), true);
                } else {
                    if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
                        robot.drivetrain.drive(x, y, trajectories.theta(robot.drivetrain, 72, -72), true);
                    } else {
                        robot.drivetrain.drive(x, y, trajectories.theta(robot.drivetrain, 72, 72), true);
                    }
                }
            } else {
                robot.drivetrain.drive(x, y, rx, false);
            }
        }
        robot.drivetrain.periodic();
        robot.spindexer.periodic();
        if (!robot.spindexer.isMoving && robot.spindexer.states.getSelectedIndex() > 2 && Objects.equals(robot.spindexer.slots.get(robot.spindexer.states.getSelectedIndex() - 3), "empty")) {
            robot.spindexer.slots.remove(robot.spindexer.states.getSelectedIndex() - 3);
            robot.spindexer.slots.add(robot.spindexer.states.getSelectedIndex() - 3, robot.colorSensor.getColour());
        }

        if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
            robot.actions.schedule(robot.flywheel.setVls(trajectories.distanceToTarget(robot.drivetrain, false)));
        } else {
            robot.actions.schedule(robot.flywheel.setVls(trajectories.distanceToTarget(robot.drivetrain, true)));
        }
        robot.flywheel.periodic();
        robot.telemetryControl.update();
        robot.actions.run();
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}