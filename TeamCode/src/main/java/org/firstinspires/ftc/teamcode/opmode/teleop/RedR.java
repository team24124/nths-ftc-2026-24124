package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

import java.util.List;

@TeleOp(name = "RedR", group = "!")
public class RedR extends OpMode {
    private Robot robot;
    private GamepadEx driver, operator;
    private TeleOpTrajectories trajectories;
    private List<LynxModule> hubs;
    private boolean alignToAT = false;
    private boolean trajectoryAlign = false;

    @Override
    public void init() {
        // Get all hubs (Control Hub internal + any Expansion Hubs)
        hubs = hardwareMap.getAll(LynxModule.class);

        // Set bulk caching mode MANUAl
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
        // MANUAL mode: bulk cache refresh happens once per loop
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

        // Enable AT alignment
        if (driver.wasJustPressed(GamepadKeys.Button.A)) { // Semi autonomous alignment mode (PD's with limelight)
            robot.limelight.setPipeline(Limelight.Pipeline.AT3);
            trajectoryAlign = false;
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) { // Pure TeleOp with ability to reset pose (MT2)
            robot.limelight.setPipeline(Limelight.Pipeline.AT4);
            trajectoryAlign = false;
            alignToAT = false;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) { // Full autonomous alignment (Same AT pipeline as semi alignment)
            robot.limelight.setPipeline(Limelight.Pipeline.AT3);
            alignToAT = false;
            trajectoryAlign = true;
        }

        // Align with Roadrunner trajectory
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            if (robot.limelight.isDetected()) {
                trajectories.poseAlign(robot.drivetrain.getDrive(), robot.limelight.ATTargetPoseFieldSpace(robot.drivetrain.getDrive().localizer.getPose()));
            }
        }

        // Reset robot pose with MT2
        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
            if (robot.limelight.isDetected() && !alignToAT && !trajectoryAlign) {
                robot.drivetrain.getDrive().localizer.setPose(robot.limelight.ATRobotPoseFieldSpace());
            }
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

        // Add if left bumper = green, right bumper = purple

        // --- Periodic calls ---
        if (!robot.drivetrain.getDrive().isBusy) { // Ensure drive isn't called during trajectory
            if (alignToAT) {
                if (robot.limelight.isDetected()) {
                    // Align heading by passing Tx value to drive
                    robot.drivetrain.drive(x, y, robot.limelight.degreeOffset(), true);
                } else {
                    // Rotate to the general direction of the target with 30 inch threshold
                    robot.drivetrain.drive(x, y, trajectories.rotation(robot.drivetrain, 72), false);
                }
            } else {
                // Pure drive
                robot.drivetrain.drive(x, y, rx, false);
            }
        }
        robot.drivetrain.periodic(); // Update position

        driver.readButtons();
        operator.readButtons();

        robot.turretBase.setHeadings(robot.drivetrain.getDrive(), robot.limelight.degreeOffset(), robot.limelight.isDetected(), true);
        robot.turretBase.periodic(); // PD loop

        robot.telemetryControl.update();
        robot.actions.run(); // Call for scheduled actions to run
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}
