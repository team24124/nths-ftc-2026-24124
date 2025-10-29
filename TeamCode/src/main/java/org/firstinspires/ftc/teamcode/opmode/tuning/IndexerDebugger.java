package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;

import java.util.List;

@Config
@TeleOp(name = "Indexer", group = "tuning")
public class IndexerDebugger extends OpMode {
    private List<LynxModule> hubs;
    private GamepadEx driver;
    private ActionScheduler actions;
    private Spindexer spindexer;
    private TelemetryControl telemetryControl;
    public static double Ks, Kp, Ki, Kd = 0;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driver = new GamepadEx(gamepad1);
        actions = ActionScheduler.INSTANCE;
        actions.init();
        spindexer = new Spindexer(hardwareMap);
        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl.subscribe(spindexer);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        spindexer.setPIDF(Kp, Ki, Kd, Ks, 0.7, 0.7);
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            actions.schedule(spindexer.shootAll());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            actions.schedule(spindexer.shootNearest());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            actions.schedule(spindexer.intakeToEmpty());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            actions.schedule(spindexer.sortTo("empty"));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            spindexer.states.previous();
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            spindexer.states.next();
        }

        spindexer.periodic();
        driver.readButtons();
        actions.run();
        telemetryControl.getTelemetry().addLine("A to shoot all\nB to shoot nearest\nX to move to an empty slot\nY to sort to colour\nPad left & right to move 1 slot");
        telemetryControl.update();
    }
}