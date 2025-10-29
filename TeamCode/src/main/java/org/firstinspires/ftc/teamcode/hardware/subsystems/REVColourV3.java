package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

import java.util.Arrays;

public class REVColourV3 implements SubsystemBase, TelemetryObservable {
    private final NormalizedColorSensor sensor1, sensor2;
    public float gain = 1;
    public REVColourV3(HardwareMap hw){
        sensor1 = hw.get(NormalizedColorSensor.class, "sensor1");
        sensor2 = hw.get(NormalizedColorSensor.class, "sensor2");
        gain(gain);
    }

    public class ColorRange {
        protected final String name;
        protected final double[] min;
        protected final double[] max;

        public ColorRange(String name, double[] min, double[] max) {
            this.name = name;
            this.min = min;
            this.max = max;
        }

        public boolean inRange(double[] color) {
            for (int i = 0; i < color.length; i++) {
                if ((color[i] <= min[i]) || (color[i] >= max[i])) {
                    return false;
                }
            }
            return true;
        }
    }

    public final ColorRange ARTIFACT_GREEN = new ColorRange(
            "green",
            new double[] {0.01, 0.04, 0.03},
            new double[] {0.019, 0.072, 0.056}
    );

    public final ColorRange ARTIFACT_PURPLE = new ColorRange(
            "purple",
            new double[] {0.017, 0.017, 0.0033},
            new double[] {0.039, 0.040, 0.07}
    );

    private double[] getColours(NormalizedColorSensor s) {
        return new double[]{(double) s.getNormalizedColors().red / s.getNormalizedColors().alpha, (double) s.getNormalizedColors().green / s.getNormalizedColors().alpha, (double) s.getNormalizedColors().blue / s.getNormalizedColors().alpha};
    }

    public String getColour() {
        if (ARTIFACT_PURPLE.inRange(getColours(sensor1)) || ARTIFACT_PURPLE.inRange(getColours(sensor2))) {
            return ARTIFACT_PURPLE.name;
        } else if (ARTIFACT_GREEN.inRange(getColours(sensor1)) || ARTIFACT_GREEN.inRange(getColours(sensor2))) {
            return ARTIFACT_GREEN.name;
        } else {
            return "empty";
        }
    }

    public void gain(float gain) {
        this.gain = gain;
        sensor1.setGain(gain);
        sensor2.setGain(gain);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Colour", Arrays.toString(getColours(sensor1)));
        telemetry.addData("Gain", gain);
    }

    @Override
    public String getName() {
        return "Colour Sensor";
    }
}