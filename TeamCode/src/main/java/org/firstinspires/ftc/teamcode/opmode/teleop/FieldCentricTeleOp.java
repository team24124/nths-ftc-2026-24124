package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.ParallelAction;
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

@TeleOp(name = "Field Centric TeleOp", group = "!")
public class FieldCentricTeleOp extends OpMode {
    private List<LynxModule> hubs;
    private Robot robot;
    private GamepadEx driver, operator;
    private TeleOpTrajectories trajectories;
    private boolean alignToAT = false;
    private boolean toggleFlywheel = false;

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

        boolean align0 = driver.isDown(GamepadKeys.Button.B);
        if (align0) {
            alignToAT = false;
            robot.drivetrain.drive(x, y, robot.drivetrain.getPosition().heading.toDouble(), true);
        }

        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(robot.intakePeriodic());
        } else if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER) && robot.spindexer.states.getSelectedIndex() > 2) {
            robot.actions.schedule(new ParallelAction(robot.intake.reverseIntake(), robot.spindexer.removeIndexed(robot.spindexer.states.getSelectedIndex() - 3)));
        } else if (robot.spindexer.isMoving) {
            robot.actions.schedule(robot.intake.runIntake());
        } else {
            robot.actions.schedule(robot.intake.stopIntake());
        }

        // --------- Operator inputs ---------
//        if (operator.wasJustPressed(GamepadKeys.Button.A)) {
//            toggleFlywheel = !toggleFlywheel;
//            if (toggleFlywheel) {
//                robot.actions.schedule(new ParallelAction(robot.flywheel.runFlywheel(), robot.spindexer.sortTo(0)));
//            } else {
//                robot.actions.schedule(robot.flywheel.stopFlywheel());
//            }
//        }
//
//        if (!robot.spindexer.isMoving && operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && robot.spindexer.states.getSelectedIndex() < 3) {
//            robot.actions.schedule(robot.spindexer.shootAll());
//        }
//
//        if (robot.flywheel.primed) {
//            if (!robot.spindexer.isMoving && operator.wasJustPressed(GamepadKeys.Button.X) && robot.spindexer.states.getSelectedIndex() < 3) {
//                robot.actions.schedule(robot.spindexer.shootAll());
//            }
//        }

        if (operator.wasJustPressed(GamepadKeys.Button.B) && !driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(robot.spindexer.sortTo(0));
        }

        if (operator.wasJustPressed(GamepadKeys.Button.A)) {
            toggleFlywheel = !toggleFlywheel;
            if (toggleFlywheel) {
                robot.actions.schedule(robot.flywheel.runFlywheel());
            } else {
                robot.actions.schedule(robot.flywheel.stopFlywheel());
            }
        }

        if (!robot.spindexer.isMoving && operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && robot.spindexer.states.getSelectedIndex() < 3) {
            robot.actions.schedule(robot.spindexer.shootAll());
        }

        if (robot.flywheel.primed) {
            if (!robot.spindexer.isMoving && operator.wasJustPressed(GamepadKeys.Button.X) && robot.spindexer.states.getSelectedIndex() < 3) {
                robot.actions.schedule(robot.spindexer.shootAll());
            }
        }

        // --- Periodic calls ---
        driver.readButtons();
        operator.readButtons();

        robot.drivetrain.periodic();
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
            } else if (!align0){
                robot.drivetrain.drive(x, y, rx, false);
            }
        }

        robot.spindexer.periodic();

        double d;
        if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
            d = trajectories.distanceToTarget(robot.drivetrain, false);
        } else {
            d = trajectories.distanceToTarget(robot.drivetrain, true);
        }
        robot.spindexer.updateDistance(d);
        robot.actions.schedule(robot.flywheel.setVls(d));
        robot.flywheel.periodic();

        robot.actions.run();
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}