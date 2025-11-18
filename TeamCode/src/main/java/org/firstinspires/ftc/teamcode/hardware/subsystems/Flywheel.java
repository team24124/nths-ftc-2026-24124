package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;
import org.firstinspires.ftc.teamcode.util.plotting.InterpLUT;

public class Flywheel implements SubsystemBase, TelemetryObservable {
    public final DcMotorEx wheel1, wheel2;
    public final Servo flap;
    public boolean powered = false;
    public boolean primed = false;
    public double targetVel = 0;
    private double distance = 0;
    private PIDF pv = new PIDF();
    private InterpLUT LUT = new InterpLUT();
    private final VoltageSensor voltageSensor;

    public Flywheel(HardwareMap hw) {
        wheel1 = hw.get(DcMotorEx.class, "wheel1"); // Connected Ehub 3
        wheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        wheel2 = hw.get(DcMotorEx.class, "wheel2"); // Connected Ehub 2
        wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        flap = hw.get(Servo.class, "flap");

        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        pv.setPV(0.0005, 0.00042);

        double[] dists = {5, 6, 7, 8}; // Inches
        double[] vels = {600, 601, 602, 603}; // Ticks/second
        LUT = new InterpLUT(dists, vels);
        LUT.setExtrapolation(InterpLUT.Extrapolation.LINEAR);
    }

    /**
     * Distance(Inches) = Distance to goal
     * Theta(Degrees from horizontal) = 45 - 4((Distance/12) - 8.5), Clamped to 40 - 80
     * Velocity(ticks/s) = tps of 5203 YellowJacket 6k rpm motor (1:1 GR, 1 x 28 tpr)
     */
    @Override
    public void periodic(){
        if (powered) {
            //targetVel = LUT.get(distance);

            //adjustFlap(distance);
            power(targetVel);
            primed = Utilities.isBetween(wheel1.getVelocity(), targetVel - 20, targetVel + 50);
        } else {
            power(0);
        }
    }

    public Action autonPeriodic() {
        return (TelemetryPacket packet) -> {
            periodic();

            return true;
        };
    }

    public Action runFlywheel() {
        return (TelemetryPacket packet) -> {
            powered = true;

            return false;
        };
    }

    public Action stopFlywheel() {
        return (TelemetryPacket packet) -> {
            powered = false;
            primed = false;

            return false;
        };
    }

    public void adjustFlap(double distance) {
        double theta = 45 - 4 * ((distance/12) - 8.5);
        theta = Range.clip(theta, 40, 80);
        flap.setPosition(1 - (theta - 40)/300); // Desired angle - servo pose 0 angle from horizontal / 300 to get servo normalized position [0, 1]
    }

    public void power(double vel) {
        // 2800 = max tps
        wheel1.setPower(pv.calculate(wheel1.getVelocity(), vel, voltageSensor.getVoltage()));
        wheel2.setPower(pv.calculate(wheel1.getVelocity(), vel, voltageSensor.getVoltage()));
    }

    public Action setVls(double distance) {
        this.distance = distance;
        return (TelemetryPacket packet) -> false;
    }

    public void setVelPID(double Kp, double Kv) {
        pv.setPV(Kp, Kv);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Moving", powered);
        telemetry.addData("Target Velocity", targetVel);
        telemetry.addData("Wheel 1 Velocity", wheel1.getVelocity());
        telemetry.addData("Wheel 2 Velocity", wheel2.getVelocity());
        telemetry.addData("Primed", primed);
        telemetry.addData("Flap Position", flap.getPosition());
    }

    @Override
    public String getName() {
        return "Flywheel";
    }
}