package org.firstinspires.ftc.teamcode.opmode.debug.tune;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.plotting.Oscillator;

import java.util.ArrayList;
import java.util.Arrays;
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
    public static double Kp = 0.0027;
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
        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl.subscribe(spindexer).subscribe(actions);
        os = new Oscillator(new Double[]{1.0, 2.0}, 1);
        os.enableOscillation(false);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        spindexer.setPD(Kp, Kd, 0.7);
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            actions.schedule(spindexer.shootOne());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            actions.schedule(spindexer.kick());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            actions.schedule(spindexer.intakeToEmpty());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            actions.schedule(spindexer.inTo("empty"));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            spindexer.states.previous();
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            spindexer.states.next();
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            actions.schedule(
                    orderedShot(new ArrayList<>(Arrays.asList("purple", "green", "purple")))
            );
        }

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            os.enableOscillation(!os.enabled());
        }
        if (os.returnSetpoint() == 1.0) {
            spindexer.states.setSelected(4);
        } else if (os.returnSetpoint() == 2.0) {
            spindexer.states.setSelected(5);
        }

        spindexer.periodic();
        driver.readButtons();
        actions.run();
        telemetryControl.getTelemetry().addData("Servo Pos", spindexer.kicker.getPosition());
        telemetryControl.update();
    }

    public Action orderedShot(List<String> pattern) {
        ElapsedTime timer = new ElapsedTime();
        final int[] counter = {1};
        return (TelemetryPacket packet) -> {
            if (counter[0] == 1) {
                spindexer.states.setSelected(spindexer.slots.indexOf(pattern.get(0)));
                removeIndexed(spindexer.slots.indexOf(pattern.get(0)));
                counter[0]++;
            }

            if (timer.seconds() < 0.4) {
                return true;
            }
            spindexer.kicker.setPosition(0.35);

            if (timer.seconds() < 0.7) {
                return true;
            }
            spindexer.kicker.setPosition(0.76);

            if (timer.seconds() < 1.4) {
                return true;
            }

            if (counter[0] == 2) {
                spindexer.states.setSelected(spindexer.slots.indexOf(pattern.get(1)));
                removeIndexed(spindexer.slots.indexOf(pattern.get(1)));
                counter[0]++;
            }

            if (timer.seconds() < 1.8) {
                return true;
            }
            spindexer.kicker.setPosition(0.35);

            if (timer.seconds() < 2.1) {
                return true;
            }
            spindexer.kicker.setPosition(0.76);

            if (timer.seconds() < 2.8) {
                return true;
            }


            if (counter[0] == 3) {
                spindexer.states.setSelected(spindexer.slots.indexOf(pattern.get(0)));
                removeIndexed(spindexer.slots.indexOf(pattern.get(0)));
                counter[0]++;
            }

            if (timer.seconds() < 3.2) {
                return true;
            }
            spindexer.kicker.setPosition(0.35);

            if (timer.seconds() < 3.5) {
                return true;
            }
            spindexer.kicker.setPosition(0.76);

            if (timer.seconds() < 4.2) {
                return true;
            }

            return false;
        };
    }

    public void removeIndexed(int i) {
        spindexer.slots.remove(i);
        spindexer.slots.add(i, "empty");
    }
}