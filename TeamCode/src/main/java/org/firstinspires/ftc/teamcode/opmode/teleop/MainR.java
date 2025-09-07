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
    private TeleOpTrajectories trajectory;
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

        robot = new Robot(hardwareMap, telemetry, true);
        robot.actions = ActionScheduler.INSTANCE;
        trajectory = TeleOpTrajectories.INSTANCE;
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
        double y = -driver.getLeftY();
        double x = driver.getLeftX();
        double rx = -driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::next));
        }

        // Enable constant AT alignment
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.limelight.setPipeline(Limelight.Pipeline.AT1);
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            alignToAT = false;
        }

        // Move to target position in front of AT
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            robot.actions.schedule(trajectory.poseAlign(robot.driveTrain.getDrive(), robot.limelight.ATTargetPoseRobotSpace()));
        }

        // Operator inputs (yet to be added)


        // Periodic calls
        if (!robot.driveTrain.getDrive().isBusy) { // Ensure drive and align aren't called during trajectory
            if (robot.limelight.isDetected() && alignToAT) {
                robot.driveTrain.align(x, y, robot.limelight.distance(), robot.limelight.degreeOffset());
            } else {
                robot.driveTrain.drive(x, y, rx);
            }
        }

        robot.telemetryControl.update();

        driver.readButtons();
        operator.readButtons();

        robot.driveTrain.periodic(); // Update position

        robot.actions.run(); // Call for scheduled actions to run
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}
