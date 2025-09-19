package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class ColourSensor implements SubsystemBase, TelemetryObservable {
    private NormalizedColorSensor colorSensor;

    public ColourSensor(HardwareMap hw) {
        colorSensor = hw.get(NormalizedColorSensor.class, "color_sensor");
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Determining the amount of red, green, and blue
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);
        telemetry.update();
    }

    @Override
    public String getName() {
        return "Colour sensor v3";
    }
}
