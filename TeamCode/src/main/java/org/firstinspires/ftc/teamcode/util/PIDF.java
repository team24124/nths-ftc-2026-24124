package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;
    private double Kg = 0; // Gravitational correction (feedforward)
    private double smoothingFactor = 0; // a can be any value from 0 < a < 1. heavier smoothing but less responsiveness towards 1
    private double integralSumLimit = 0; // Integral cap to prevent unnecessary accumulation

    private double target = 0;
    private double lastTarget = target;
    private double integralSum = 0;
    private double derivative = 0;
    private double error = 0;
    private double prevError = 0;
    private double previousFilterEstimate = 0;
    private double currentFilterEstimate = 0;
    private double motorTPR = 0;

    private ElapsedTime timer = new ElapsedTime();

    // Feedback and feedforward
    public void setPIDF(double Kp, double Ki, double Kd, double Kg, double smoothingFactor, double integralSumLimit, double motorTPR) {
        timer.reset();
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kg = Kg;
        this.smoothingFactor = smoothingFactor;
        this.integralSumLimit = integralSumLimit;
    }

    // Feedback only
    public void setPID(double Kp, double Ki, double Kd, double smoothingFactor, double integralSumLimit, double motorTPR) {
        timer.reset();
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.smoothingFactor = smoothingFactor;
        this.integralSumLimit = integralSumLimit;
    }

    // Removed integral for uncertain error
    public void setPD(double Kp, double Kd, double smoothingFactor, double motorTPR) {
        timer.reset();
        this.Kp = Kp;
        this.Kd = Kd;
        this.smoothingFactor = smoothingFactor;
    }

    // Output power
    public double calculate(double position, double target, double voltage) {
        // Time
        double dt = timer.seconds();
        timer.reset();
        if (dt > 0.1) dt = 0.1;

        // Error
        error = target - position;

        // Integral setters & limiters
        if (lastTarget != target) {
            integralSum = 0;
            lastTarget = target;
        }
        integralSum += (error * dt);
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }

        // Derivative setter & sensor noise filters
        double rawDerivative = (error - prevError) / dt;
        currentFilterEstimate = (smoothingFactor * previousFilterEstimate) + (1 - smoothingFactor) * rawDerivative;
        derivative = currentFilterEstimate;
        previousFilterEstimate = currentFilterEstimate;

        // Angle getter
        double radians = 2 * Math.PI * (position / motorTPR); // Assumes an arm is initialized horizontally, TPR is gear ratio * 28, goBILDA 312rpm 5203 yellowjacket motor

        // Output calculator
        double out = ((Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Math.cos(radians) * Kg)) * (12.0 / Math.max(voltage, 8.0));
        out = Math.max(-1, Math.min(1, out));

        // Output & error setter
        prevError = error;
        return out;
    }
}