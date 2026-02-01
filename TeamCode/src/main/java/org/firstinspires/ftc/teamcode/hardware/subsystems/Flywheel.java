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
    private PIDF pv = new PIDF();
    double[] dists = {40, 50, 60, 70, 80, 90, 100, 150}; // Inches
    double[] vels = {1050, 1070, 1120, 1150, 1260, 1310, 1340, 1590}; // Ticks/second
    private final VoltageSensor voltageSensor;
    public InterpLUT lut = new InterpLUT(dists, vels);
    private double distance = 1;
    public double targetVel = lut.get(distance);

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

        pv.setPV(0.004, 0.00042);

        lut.setExtrapolation(InterpLUT.Extrapolation.LINEAR);
    }

    /**
     * Distance(Inches) = Distance to goal
     * Velocity(ticks/s) = tps of 5203 YellowJacket 6k rpm motor (1:1 GR, 1 x 28 tpr)
     */
    @Override
    public void periodic(){
        adjustFlap(distance);
        targetVel = lut.get(distance);
        if (distance > 120) {
            setVelPID(0.0007, 0.000475);
        }
        if (powered) {
            power(targetVel);
            primed = Utilities.isBetween(wheel1.getVelocity(), targetVel -100, targetVel + 70);
        } else {
            wheel1.setPower(0);
            wheel2.setPower(0);
            primed = false;
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
        // 0.94 is extended 0.18 is lowest
        flap.setPosition(Range.clip(0.48 + ((distance - 40) * 0.0105589041) - Range.clip((targetVel - wheel1.getVelocity()) / 750, 0, 0.3), 0.34, 0.94));
    }

    public void power(double vel) {
        // 2800 = max tps
        wheel1.setPower(pv.calculate(wheel1.getVelocity(), vel, voltageSensor.getVoltage()));
        wheel2.setPower(pv.calculate(wheel1.getVelocity(), vel, voltageSensor.getVoltage()));
    }

    public Action setVls(double d) {
        return (TelemetryPacket packet) -> {
            distance = d;
            return false;
        };
    }

    public void setVelPID(double Kp, double Kv) {
        pv.setPV(Kp, Kv);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel Moving", powered);
        telemetry.addData("Flywheel LUT Value", lut.get(distance));
        telemetry.addData("Flywheel Target Velocity", targetVel);
        telemetry.addData("Flywheel 1 Velocity", wheel1.getVelocity());
        telemetry.addData("Flywheel 2 Velocity", wheel2.getVelocity());
        telemetry.addData("Flywheel Primed", primed);
        telemetry.addData("Flywheel Flap Position", flap.getPosition());
    }

    @Override
    public String getName() {
        return "Flywheel";
    }
}