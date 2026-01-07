package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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

@TeleOp(name = "Robot Centric TeleOp", group = "!")
public class RobotCentricTeleOp extends OpMode {
    private List<LynxModule> hubs;
    private Robot robot;
    private GamepadEx driver, operator;
    private TeleOpTrajectories trajectories;
    private boolean alignToAT = false;
    private boolean toggleFlywheel = false;

    @Override
    public void init() {
        // Get all hubs (Control Hub internal + any Expansion Hubs)
        hubs = hardwareMap.getAll(LynxModule.class);

        // Set bulk caching mode MANUAl
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        trajectories = TeleOpTrajectories.INSTANCE;
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry, true);
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
        // MANUAL mode: bulk cache refresh happens once per loop
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // --------- Driver inputs ---------
        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            robot.drivetrain.toggleSpeeds();
        }

        // Enable semi autonomous AT alignment (PD with limelight)
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            alignToAT = !alignToAT;
        }

        // Align heading for parking
        boolean align0 = driver.isDown(GamepadKeys.Button.B);
        if (align0) {
            alignToAT = false;
            robot.drivetrain.drive(x, y, robot.drivetrain.getPosition().heading.toDouble(), true);
        }

        // Reset PoseStorage (only if necessary)
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
                robot.drivetrain.getDrive().localizer.setPose(new Pose2d(0, 0, 0));
                PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
            } else {
                robot.drivetrain.getDrive().localizer.setPose(new Pose2d(0, 0, 0)) ;
                PoseStorage.currentAlliance = PoseStorage.Alliance.RED;
            }
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
        if (operator.wasJustPressed(GamepadKeys.Button.A)) {
            toggleFlywheel = !toggleFlywheel;
            if (toggleFlywheel) {
                robot.actions.schedule(new ParallelAction(robot.flywheel.runFlywheel(), robot.spindexer.sortTo(0)));
            } else {
                robot.actions.schedule(robot.flywheel.stopFlywheel());
            }
        }

        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.actions.schedule(robot.spindexer.sortTo(0));
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

        if (!robot.drivetrain.getDrive().isBusy) { // Ensure drive isn't called during trajectory
            if (alignToAT) {
                if (robot.limelight.isDetected()) {
                    // Align heading by passing Tx value to drive
                    robot.drivetrain.drive(x, y, Math.toRadians(robot.limelight.degreeOffset()), true);
                } else {
                    // Rotate with position and trig. Remember Y is positive to the left
                    if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
                        robot.drivetrain.drive(x, y, trajectories.theta(robot.drivetrain, 72, -72), true);
                    } else {
                        robot.drivetrain.drive(x, y, trajectories.theta(robot.drivetrain, 72, 72), true);
                    }
                }
            } else if (!align0){
                // Pure drive
                robot.drivetrain.drive(x, y, rx, false);
            }
        }
        robot.drivetrain.periodic(); // Update position
        robot.spindexer.periodic();
        if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
            robot.actions.schedule(robot.flywheel.setVls(trajectories.distanceToTarget(robot.drivetrain, false)));
        } else {
            robot.actions.schedule(robot.flywheel.setVls(trajectories.distanceToTarget(robot.drivetrain, true)));
        }
        robot.flywheel.periodic();

        if (!robot.spindexer.isMoving && robot.spindexer.states.getSelectedIndex() > 2 && Objects.equals(robot.spindexer.slots.get(robot.spindexer.states.getSelectedIndex() - 3), "empty")) {
            robot.spindexer.slots.remove(robot.spindexer.states.getSelectedIndex() - 3);
            robot.spindexer.slots.add(robot.spindexer.states.getSelectedIndex() - 3, robot.colorSensor.getColour());
        }

        robot.actions.run(); // Call for scheduled actions to run
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}