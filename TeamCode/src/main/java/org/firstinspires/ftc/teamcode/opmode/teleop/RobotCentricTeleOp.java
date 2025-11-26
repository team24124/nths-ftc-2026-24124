package org.firstinspires.ftc.teamcode.opmode.teleop;

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
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;

@TeleOp(name = "Robot Centric TeleOp", group = "!")
public class RobotCentricTeleOp extends OpMode {
    private Robot robot;
    private GamepadEx driver, operator;
    private TeleOpTrajectories trajectories;
    private List<LynxModule> hubs;
    private boolean alignToAT = false;

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


        // Startup actions
        if (!robot.intake.toggled) {
            robot.intake.toggled = true;
        }
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

        // Align with Roadrunner trajectory
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            Pose2d targetPose;
            alignToAT = false;
            if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
                targetPose = new Pose2d(-40, -30, Math.toRadians(180));
            } else {
                targetPose = new Pose2d(-40, 30, Math.toRadians(180));
            }
            robot.actions.schedule(trajectories.poseAlign(robot.drivetrain.getDrive(), targetPose));
        }

        // Reset robot pose with MT2 and heading
//        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
//            if (robot.limelight.isDetected() && !alignToAT && !trajectoryAlign) {
//                robot.drivetrain.getDrive().localizer.setPose(robot.limelight.ATRobotPoseFieldSpace());
//            }
//        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            Vector2d current = robot.drivetrain.getDrive().localizer.getPose().position;
            robot.drivetrain.getDrive().localizer.setPose(new Pose2d(current, 0));
        }

        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(robot.intakePeriodic());
        } else {
            robot.actions.schedule(robot.intake.stopIntake());
        }

        // --------- Operator inputs ---------
        if (operator.isDown(GamepadKeys.Button.A)) {
            robot.actions.schedule(robot.flywheel.runFlywheel());
        } else {
            robot.actions.schedule(robot.flywheel.stopFlywheel());
        }

        if (operator.isDown(GamepadKeys.Button.X) && robot.flywheel.primed) {
            robot.actions.schedule(robot.spindexer.shootOne());
        }
        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && robot.flywheel.primed && robot.spindexer.slots.contains("purple")) {
            robot.actions.schedule(robot.shootColor("purple"));
        }
        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && robot.flywheel.primed && robot.spindexer.slots.contains("green")) {
            robot.actions.schedule(robot.shootColor("green"));
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
            } else {
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
        robot.telemetryControl.update();
        robot.actions.run(); // Call for scheduled actions to run
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}