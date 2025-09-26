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

@TeleOp(name = "BlueR", group = "!")
public class BlueR extends OpMode {
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
        double y = Math.abs(driver.getLeftY()) > 0.05 ? driver.getLeftY() : 0;
        double x = Math.abs(driver.getLeftX()) > 0.05 ? driver.getLeftX() : 0;
        double rx = Math.abs(driver.getRightX()) > 0.05 ? driver.getRightX() : 0;

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::next));
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
                trajectories.poseAlign(robot.driveTrain.getDrive(), robot.limelight.ATTargetPoseFieldSpace(robot.driveTrain.getDrive().localizer.getPose()));
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
            if (robot.limelight.isDetected() && !alignToAT && !trajectoryAlign) {
                robot.driveTrain.getDrive().localizer.setPose(robot.limelight.ATRobotPoseFieldSpace());
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

        // --- Periodic calls ---
        if (!robot.driveTrain.getDrive().isBusy) {
            if (alignToAT) {
                if (robot.limelight.isDetected()) {
                    robot.driveTrain.drive(x, y, robot.limelight.degreeOffset(), true);
                } else {
                    robot.driveTrain.drive(x, y, trajectories.rotation(robot.driveTrain, -72), false);
                }
            } else {
                robot.driveTrain.drive(x, y, rx, false);
            }
        }
        robot.driveTrain.periodic();

        driver.readButtons();
        operator.readButtons();

        robot.turretBase.setHeadings(robot.driveTrain.getDrive(), robot.limelight.degreeOffset(), robot.limelight.isDetected(), false);
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
