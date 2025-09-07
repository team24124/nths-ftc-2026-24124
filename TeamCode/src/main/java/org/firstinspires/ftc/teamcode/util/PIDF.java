package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;
    private double Kf = 0;
    private double a = 0;
    private double integralSumLimit = 0;

    private double target = 0;
    private double lastTarget = target;
    private double integralSum = 0;
    private double derivative = 0;
    private double error = 0;
    private double prevError = 0;
    private double previousFilterEstimate = 0;
    private double currentFilterEstimate = 0;
    private double motorTPR = 537.6;

    private ElapsedTime timer = new ElapsedTime();

    // Feedback and feedforward
    public void setPIDF(double Kp, double Ki, double Kd, double Kf, double a, double integralSumLimit) {
        timer.reset();
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.a = a;
        this.integralSumLimit = integralSumLimit;
    }

    // Feedback only
    public void setPID(double Kp, double Ki, double Kd, double a, double integralSumLimit) {
        timer.reset();
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.a = a;
        this.integralSumLimit = integralSumLimit;
    }

    // Output power
    public double calculate(double position, double target, double voltage) {
        double dt = timer.seconds();
        timer.reset();
        if (dt > 0.1) dt = 0.1;

        error = target - position;

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

        double rawDerivative = (error - prevError) / dt;
        currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * rawDerivative;
        derivative = currentFilterEstimate;

        double radians = 2 * Math.PI * (position / motorTPR);

        double out = ((Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Math.cos(radians) * Kf)) * (12.0 / Math.max(voltage, 8.0));
        out = Math.max(-1, Math.min(1, out));

        prevError = error;
        return out;
    }
}