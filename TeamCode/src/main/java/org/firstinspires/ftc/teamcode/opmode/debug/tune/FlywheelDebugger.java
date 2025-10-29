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

import java.util.List;

@Config
@TeleOp(name = "FlyWheel", group = "tuning")
public class FlywheelDebugger extends OpMode {
    private GamepadEx driver;
    private List<LynxModule> hubs;
    private Flywheel flywheel;
    private TeleOpTrajectories trajectories;
    private Drivetrain drivetrain;
    private ActionScheduler actions;
    private TelemetryControl telemetryControl;
    public static double Kp, Kv = 0;

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
        trajectories = TeleOpTrajectories.INSTANCE;
        drivetrain = new FieldCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));
        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl.subscribe(flywheel);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        flywheel.setVelPID(Kp, Kv);

        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            flywheel.adjustFlap(trajectories.distanceToTarget(drivetrain, true));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            actions.schedule(flywheel.runFlywheel());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            actions.schedule(flywheel.stopFlywheel());
        }

        flywheel.setVls(trajectories.distanceToTarget(drivetrain, true));
        flywheel.periodic();
        drivetrain.periodic();
        driver.readButtons();
        actions.run();
        telemetryControl.update();
    }
}
