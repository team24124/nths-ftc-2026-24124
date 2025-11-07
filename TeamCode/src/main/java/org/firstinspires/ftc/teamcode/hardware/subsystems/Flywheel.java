package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

public class Flywheel implements SubsystemBase, TelemetryObservable {
    public final DcMotorEx wheel1, wheel2;
    private final Servo flap;
    public boolean powered = false;
    private double targetVel = 0;
    private double distance = 0;
    private PIDF pv = new PIDF();
    private final VoltageSensor voltageSensor;

    public Flywheel(HardwareMap hw) {
        wheel1 = hw.get(DcMotorEx.class, "wheel1");
        wheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        wheel2 = hw.get(DcMotorEx.class, "wheel2");
        wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        flap = hw.get(Servo.class, "flap");

        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        pv.enableFilters(false);
        pv.setPV(0.0005, 0.00042);
    }

    /**
     * Distance(feet) = Distance in inches / 12
     * Theta(Degrees from horizontal) = 45 - 4(Distance - 8.5)
     * Velocity(ft/s) = 11 + 1.15 * Distance + 0.023 * Distance^2
     * (ft/s) * 56.596 = tps of 5203 YellowJacket 6k rpm motor (1:1 GR, 1 x 28 tpr)
     */
    @Override
    public void periodic(){
        if (powered) {
            distance /= 12;
            targetVel = 11 + 1.15 * distance + 0.023 * Math.pow(distance, 2); // ft/s
            targetVel *= 56.596; // Ticks/s

            //adjustFlap(distance);
            power(targetVel);
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

            return false;
        };
    }

    public void adjustFlap(double distance) {
        double theta = 45 - 4 * (distance - 8.5);
        flap.setPosition((theta - 45)/300); // Desired angle - servo pose 0 angle from horizontal / 300 to get servo normalized position [0, 1]
    }

    public void power(double vel) {
        // 2800 = max tps
        wheel1.setPower(pv.calculate(wheel1.getVelocity(), vel, voltageSensor.getVoltage()));
        wheel2.setPower(pv.calculate(wheel1.getVelocity(), vel, voltageSensor.getVoltage()));
    }

    public void setVls(double distance) {
        this.distance = distance;
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
    }

    @Override
    public String getName() {
        return "Flywheel";
    }
}