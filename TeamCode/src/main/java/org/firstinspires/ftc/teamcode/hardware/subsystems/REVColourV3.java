package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class REVColourV3 implements SubsystemBase, TelemetryObservable {
    float ACCEPTABLE_THRESHOLD = 0.1f;
    public NormalizedColorSensor colorSensor;

    public REVColourV3(HardwareMap hw){
        colorSensor = hw.get(NormalizedColorSensor.class, "colorSensor");
    }

    public float[] getHSVColors(){
        int r = (int) colorSensor.getNormalizedColors().red * 255;
        int g = (int) colorSensor.getNormalizedColors().green * 255;
        int b = (int) colorSensor.getNormalizedColors().blue * 255;

        float[] hsvValues = new float[3];
        Color.RGBToHSV(r, g, b, hsvValues);
        return hsvValues;
    }

    public boolean isColorGreen(){
        float[] hsvColor = getHSVColors();
        float value = hsvColor[0];

        return (value == 120);
    }
    public boolean isColorPurple(){
        float[] hsvColor = getHSVColors();
        float value = hsvColor[0];

        return (value == 240);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Is Purple", isColorPurple());
        telemetry.addData("Is Green", isColorGreen());
    }

    @Override
    public String getName() {
        return "Colour Sensor";
    }
}