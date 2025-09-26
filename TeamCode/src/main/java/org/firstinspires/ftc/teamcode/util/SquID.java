package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

/**
 * Implements a non-linear proportional controller using the square of the error.
 * This provides high power for large errors and finer, smoother control as the
 * robot approaches the target, helping to reduce overshoot and oscillation.
 */
public class SquID {
   /*
    * Minimum power (feedforward) to apply to overcome static friction
    * Applied in the direction of the error
    * Recommended value ~ 0.05 - 0.1
    */
    private double minPower;

    /*
     * Proportional gain that scales the overall power. Tune this for responsiveness.
     * The stretch factor of a parabolic function controls where y (power) < 1.
     * Lowering the stretch factor causes the motor to hold full power closer to the targeted error value (0).
     * Recommended range for a 537.6 TPR motor is around 0.0001 where power drops below 1 at error = +-100 and starts smoothing.
     */
    private double stretchFactor;

    public SquID(double stretchFactor, double minPower) {
        this.stretchFactor = stretchFactor;
        this.minPower = minPower;
    }

    public void setSquID(double stretchFactor, double minPower) {
        this.stretchFactor = stretchFactor;
        this.minPower = minPower;
    }

    /**
     * Calculates the motor power based on the current position and target.
     *
     * @param position The current position of the motor (encoder ticks).
     * @param target The target position for the motor (encoder ticks).
     * @param voltage The current battery voltage, used for compensation.
     * @return The calculated motor power, clipped to the range [-1.0, 1.0].
     */
    public double calculate(int position, int target, double voltage) {
        // Remaining distance to target
        double error = target - position;

        // Using a small tolerance to prevent jitter
        if (Math.abs(error) < 4) {
            return 0.0;
        }

        // The core of the controller: power is proportional to the square of the absolute error
        // Non-linear function where power is y (inverted below x = 0) and error is x
        double power = stretchFactor * (error * Math.abs(error));
        power += Math.copySign(minPower, error);

        // Clip the final power to the valid motor range of [-1.0, 1.0]
        power = Range.clip(power, -1.0, 1.0);

        // Voltage compensation. A lower bound of 8.0V prevents excessive scaling if the voltage reading is abnormal.
        return power * (12.0 / Math.max(voltage, 8.0));
    }
}