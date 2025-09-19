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

@TeleOp(name = "MainR", group = "!")
public class MainR extends OpMode {
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

        // Driver inputs
        double y = Math.abs(-driver.getLeftY()) > 0.05 ? -driver.getLeftY() : 0;
        double x = Math.abs(driver.getLeftX()) > 0.05 ? driver.getLeftX() : 0;
        double rx = Math.abs(-driver.getRightX()) > 0.05 ? -driver.getRightX() : 0;

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::next));
        }

        // Enable constant AT alignment
        if (driver.wasJustPressed(GamepadKeys.Button.A)) { // Semi autonomous alignment mode (PIDs with limelight)
            robot.limelight.setPipeline(Limelight.Pipeline.AT2);
            trajectoryAlign = false;
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) { // Pure TeleOp with ability to reset pose (MT2)
            robot.limelight.setPipeline(Limelight.Pipeline.AT4);
            trajectoryAlign = false;
            alignToAT = false;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) { // Full autonomous alignment (Same AT pipeline as semi alignment)
            robot.limelight.setPipeline(Limelight.Pipeline.AT2);
            alignToAT = false;
            trajectoryAlign = true;
        }

        // Align with Roadrunner trajectory
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            if (robot.limelight.isDetected()) {
                trajectories.poseAlign(robot.driveTrain.getDrive(), robot.limelight.ATTargetPoseFieldSpace(robot.driveTrain.getDrive().localizer.getPose()));
            }
        }

        // Reset robot pose with MT2
        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
            if (robot.limelight.isDetected() && !alignToAT && !trajectoryAlign) {
                robot.driveTrain.getDrive().localizer.setPose(robot.limelight.ATRobotPoseFieldSpace());
            }
        }

        // Operator inputs (yet to be added)


        // Periodic calls
        if (!robot.driveTrain.getDrive().isBusy) { // Ensure drive isn't called during trajectory
            if (alignToAT) {
                if (robot.limelight.isDetected()) {
                    robot.driveTrain.drive(x, y, robot.limelight.degreeOffset(), true);
                } else {
                    robot.driveTrain.drive(x, y, trajectories.rotation(robot.driveTrain, 72), false); // TODO second teleop with reversed 72
                }
            } else {
                robot.driveTrain.drive(x, y, rx, false);
            }
        }

        robot.telemetryControl.update();

        driver.readButtons();
        operator.readButtons();

        robot.turretBase.setHeadings(robot.driveTrain.getDrive(), robot.limelight.degreeOffset(), robot.limelight.isDetected(), true);

        robot.driveTrain.periodic(); // Update position
        robot.turretBase.periodic(); // PD loop

        robot.actions.run(); // Call for scheduled actions to run
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}
