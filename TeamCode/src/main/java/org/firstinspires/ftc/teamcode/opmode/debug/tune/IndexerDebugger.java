package org.firstinspires.ftc.teamcode.opmode.debug.tune;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;
import org.firstinspires.ftc.teamcode.util.plotting.Oscillator;

import java.util.List;

@Config
@TeleOp(name = "Indexer", group = "tuning")
public class IndexerDebugger extends OpMode {
    private List<LynxModule> hubs;
    private GamepadEx driver;
    private ActionScheduler actions;
    private Spindexer spindexer;
    private TelemetryControl telemetryControl;
    private Oscillator os;
    private Intake intake;
    public static double Kp = 0.003;
    public static double Kd = 0.000001;

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
        intake = new Intake(hardwareMap);
        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl.subscribe(spindexer).subscribe(actions);
        os = new Oscillator(new Double[]{1.0, 2.0}, 1);
        os.enableOscillation(false);
        actions.schedule(intake.toggleIntake(true));
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        spindexer.setPD(Kp, Kd, 0);
        if (!spindexer.isMoving && driver.wasJustPressed(GamepadKeys.Button.Y) && spindexer.states.getSelectedIndex() < 3) {
            actions.schedule(spindexer.shootAll());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            actions.schedule(spindexer.inTo("empty"));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            actions.schedule(spindexer.sortTo(0));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            spindexer.states.previous();
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            spindexer.states.next();
        }

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            os.enableOscillation(!os.enabled());
        }
        if (os.returnSetpoint() == 1.0) {
            spindexer.states.setSelected(4);
        } else if (os.returnSetpoint() == 2.0) {
            spindexer.states.setSelected(5);
        }
//        if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
//            spindexer.spindexer.setPower(0.5);
//        } else if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
//            spindexer.spindexer.setPower(-0.5);
//        } else {
            spindexer.periodic();
//        }

//        if (spindexer.spindexer.getPower() > 0.2 || spindexer.spindexer.getPower() < -0.2) {
//            actions.schedule(intake.runIntake());
//        } else {
//            actions.schedule(intake.stopIntake());
//        }

        driver.readButtons();
        actions.run();
        telemetryControl.update();
    }
}