package org.firstinspires.ftc.teamcode.util.plotting;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ArraySelect;

public class Oscillator {
    private final ArraySelect<Double> setpoints;
    private double seconds;
    private boolean oscillate = false;
    private ElapsedTime timer;

    public Oscillator(Double[] setpoints, double oscillationPeriodSeconds) {
        timer = new ElapsedTime();

        this.setpoints = new ArraySelect<>(setpoints);
        this.seconds = oscillationPeriodSeconds;
    }

    public double returnSetpoint() {
        double time = timer.seconds();
        if (time >= seconds && oscillate) {
            timer.reset();
            if (setpoints.getSelectedIndex() + 1 == getSetpoints().length) {
                return setpoints.moveSelection(-getSetpoints().length + 1).getSelected();
            }
            return setpoints.moveSelection(1).getSelected();
        } else {
            return setpoints.getSelected();
        }
    }

    public Double[] getSetpoints() {
        return setpoints.getAllOptions();
    }

    public void setOscillationPeriod(double seconds) {
        this.seconds = seconds;
    }

    public void enableOscillation(boolean enable) {
        oscillate = true;
    }
}