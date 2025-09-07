package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name="Arm Debugger", group="test")
public class ArmDebugger extends OpMode {
    private DcMotorEx arm;
    private VoltageSensor voltageSensor;
    private List<LynxModule> hubs;


    // --- tune ---
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0; // Gravitational correction (feedforward)
    public static double a = 0.74; // a can be anything from 0 < a < 1. heavier smoothing but less responsiveness towards 1
    public static double integralSumLimit = 0.8;

    // Internal math
    private int target = 0;
    private double lastTarget = target;
    private double integralSum = 0;
    private double derivative = 0;
    private int error = 0;
    private double prevError = 0;
    private final double motorTPR = 537.6;

    private double previousFilterEstimate = 0;
    private double currentFilterEstimate = 0;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();

    // Radical oscillations
    int timeLagCount = 0;
    int integralOvershoot = 0;
    int integralUndershoot = 0;


    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        timer.reset();
        timer2.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // Time
        double dt = timer.seconds();
        timer.reset();
        if (dt > 0.1) {
            dt = 0.1;
            timeLagCount += 1;
        }

        // Target setter
        if (timer2.seconds() > 6 && target == 400) {
            target = 2000;
            timer2.reset();
        } else if (timer2.seconds() > 6 && target == 2000) {
            target = 400;
            timer2.reset();
        }

        // Error
        error = target - arm.getCurrentPosition();

        // IntegralSum setter & limiters
        if (lastTarget != target) {
            integralSum = 0;
            lastTarget = target;
        }
        integralSum += (error * dt);
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
            integralOvershoot += 1;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
            integralUndershoot += 1;
        }

        // Derivative setter & sensor noise filters
        currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * (error - prevError);
        previousFilterEstimate = currentFilterEstimate;
        derivative = currentFilterEstimate / dt;

        // Angle getter
        double radians = 2 * Math.PI * ((double) arm.getCurrentPosition() / motorTPR); // Assumes arm is initialized horizontally, TPR is gear ratio * 28, goBILDA 312rpm 5203 yellowjacket motor

        // Output calculator
        double out = ((Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Math.cos(radians) * Kf)) * (12.0 / Math.max(voltageSensor.getVoltage(), 5.0));
        out = Math.max(-1, Math.min(1, out));

        // Output & error setter
        arm.setPower(out);
        prevError = error;

        // Telemetry
        telemetry.addData("Position", arm.getCurrentPosition());
        telemetry.addData("Target", target);
        telemetry.addData("Time lag count", timeLagCount);
        telemetry.addData("Integral overshoot count", integralOvershoot);
        telemetry.addData("Integral undershoot count", integralUndershoot);
        telemetry.addData("Output", out);
    }
}