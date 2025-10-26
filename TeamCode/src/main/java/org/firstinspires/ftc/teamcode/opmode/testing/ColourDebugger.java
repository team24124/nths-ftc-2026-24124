package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.REVColourV3;

import java.util.List;

@TeleOp(name = "REVColourV3", group = "test")
public class ColourDebugger extends OpMode {
    private List<LynxModule> hubs;

    private REVColourV3 colorSensor;

    @Override
    public void init() {
        // Get all hubs (Control Hub internal + any Expansion Hubs)
        hubs = hardwareMap.getAll(LynxModule.class);

        // Set bulk caching mode MANUAl
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        colorSensor = new REVColourV3(hardwareMap);
        colorSensor.colorSensor.setGain(15f);
    }

    @Override
    public void loop() {
        // MANUAL mode: bulk cache refresh happens once per loop
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        telemetry.addData("red", "%.3f", colorSensor.colorSensor.getNormalizedColors().red / colorSensor.colorSensor.getNormalizedColors().alpha);
        telemetry.addData("green", "%.3f", colorSensor.colorSensor.getNormalizedColors().green / colorSensor.colorSensor.getNormalizedColors().alpha);
        telemetry.addData("blue", "%.3f", colorSensor.colorSensor.getNormalizedColors().blue / colorSensor.colorSensor.getNormalizedColors().alpha);
        telemetry.addData("Is Green", colorSensor.isColorGreen());
        telemetry.addData("Is Purple", colorSensor.isColorPurple());
        telemetry.addData("HSV Value:", colorSensor.getHSVColors()[0]);
        telemetry.addData("Normalized G:", colorSensor.colorSensor.getNormalizedColors().green * 255);
    }
}
