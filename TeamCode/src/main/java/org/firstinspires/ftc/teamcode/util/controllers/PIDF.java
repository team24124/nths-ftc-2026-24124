package org.firstinspires.ftc.teamcode.util.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    private double Kp, Ki, Kd, Kg, Ks = 0; // Proportional, integral, derivative, gravitational correction (feedforward), friction compensation
    private double smoothingFactor = 0; // sf can be any value from 0 < sf < 1. Heavier smoothing but less responsiveness towards 1
    private double integralSumLimit = 0; // Integral cap to prevent unnecessary accumulation

    private double lastTarget, integralSum, prevError, previousFilterEstimate = 0;
    private double motorTPR = 1;

    private ElapsedTime timer = new ElapsedTime();

    // Feedback and feedforward (gravity)
    public void setPIDG(double Kp, double Ki, double Kd, double Kg, double smoothingFactor, double integralSumLimit, double motorTPR) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kg = Kg;
        this.smoothingFactor = smoothingFactor;
        this.integralSumLimit = integralSumLimit;
        this.motorTPR = motorTPR;
    }

    // Feedback and feedforward (friction)
    public void setPIDF(double Kp, double Ki, double Kd, double Ks, double smoothingFactor, double integralSumLimit) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Ks = Ks;
        this.smoothingFactor = smoothingFactor;
        this.integralSumLimit = integralSumLimit;
    }

    // Feedback only
    public void setPID(double Kp, double Ki, double Kd, double smoothingFactor, double integralSumLimit) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.smoothingFactor = smoothingFactor;
        this.integralSumLimit = integralSumLimit;
    }

    // Removed integral from feedback for unstable error
    public void setPD(double Kp, double Kd, double smoothingFactor) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.smoothingFactor = smoothingFactor;
    }

    // Outputs processed power
    public double calculate(double position, double target, double voltage) {
        // Time
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        // Error
        double error = target - position;

        // Integral setters & limiters
        if (lastTarget != target) {
            integralSum = 0;
            lastTarget = target;
        }
        integralSum += (error * dt);
        if (Math.abs(integralSum) > integralSumLimit) {
            integralSum = integralSumLimit * Math.signum(integralSum);
        }

        // Derivative setter & sensor noise filters
        double rawDerivative = (error - prevError) / dt;
        double derivative = (smoothingFactor * previousFilterEstimate) + (1 - smoothingFactor) * rawDerivative;
        previousFilterEstimate = derivative;

        // Angle getter
        double radians = 2 * Math.PI * (position / motorTPR); // Assumes an arm is initialized horizontally, TPR is gear ratio * 28, goBILDA 312rpm 5203 yellowjacket motor

        double staticFriction = (Math.abs(error) > 2) ? Ks * Math.signum(error) : 0; // Give value to friction compensation if error is above tolerance of 2 encoder ticks

        // Output calculator
        double out = ((Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Math.cos(radians) * Kg) + staticFriction) * (12.0 / Math.max(voltage, 8.0));
        out = Math.max(-1, Math.min(1, out));

        // Output & error setter
        prevError = error;
        return out;
    }
}