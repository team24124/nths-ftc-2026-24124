package org.firstinspires.ftc.teamcode.opmode.debug.tune;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;
import org.firstinspires.ftc.teamcode.util.plotting.InterpHermiteLUT;
import org.firstinspires.ftc.teamcode.util.plotting.InterpLUT;

import java.util.List;

@Config
@TeleOp(name = "FlyWheel", group = "tuning")
public class FlywheelDebugger extends OpMode {
    private GamepadEx driver;
    private List<LynxModule> hubs;
    private Flywheel flywheel;
    private ActionScheduler actions;
    private TelemetryControl telemetryControl;
    public static double Kp = 0.0005;
    public static double Kv = 0.00042;
    public static double velocity = 1;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        actions = ActionScheduler.INSTANCE;
        actions.init();
        driver = new GamepadEx(gamepad1);
        flywheel = new Flywheel(hardwareMap);
        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl.subscribe(flywheel);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        flywheel.setVelPID(Kp, Kv);

        //if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
        //    flywheel.adjustFlap(distance);
        //}
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            actions.schedule(flywheel.runFlywheel());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            actions.schedule(flywheel.stopFlywheel());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            flywheel.wheel1.setPower(0.3);
        }
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            flywheel.wheel2.setPower(0.3);
        }
        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            flywheel.wheel1.setPower(0);
            flywheel.wheel2.setPower(0);
        }

        if (flywheel.powered) {
            flywheel.power(velocity); // Enter ticks/second
        } else {
            flywheel.power(0);
        }

        driver.readButtons();
        actions.run();
        telemetryControl.update();
    }
}
