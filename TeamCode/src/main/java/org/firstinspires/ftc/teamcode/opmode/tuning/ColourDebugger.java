package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.REVColourV3;

import java.util.List;

@TeleOp(name = "Colour", group = "test")
public class ColourDebugger extends OpMode {
    private List<LynxModule> hubs;
    private GamepadEx driver;
    private REVColourV3 colorSensor;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driver = new GamepadEx(gamepad1);

        colorSensor = new REVColourV3(hardwareMap);
    }

    @Override
    public void loop() {
        // MANUAL mode: bulk cache refresh happens once per loop
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            colorSensor.gain += 1;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            colorSensor.gain -= 1;
        }

        driver.readButtons();

        telemetry.addLine(colorSensor.getColour());
        colorSensor.updateTelemetry(telemetry);
    }
}
