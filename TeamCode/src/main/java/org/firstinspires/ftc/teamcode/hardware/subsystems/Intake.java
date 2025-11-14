package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class Intake implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx intake;
    public boolean powered = false;
    public boolean toggled = true;
    private double targetVel = 0;

    public Intake(HardwareMap hw) {
        intake = hw.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public Action runIntake() {
        return (TelemetryPacket packet) -> {
            targetVel = -((double) 312 /60)*360; // 1872 degrees per second
            intake.setVelocity(targetVel * (537.6/360)); // Degree to tick conversion
            powered = true;

            return false;
        };
    }

    public Action stopIntake() {
        return (TelemetryPacket packet) -> {
            targetVel = 0;
            intake.setVelocity(0);
            powered = false;

            return false;
        };
    }

    public Action toggleIntake() {
        return (TelemetryPacket packet) -> {
            toggled = !toggled;

            return false;
        };
    }

    public Action toggleIntake(boolean tog) {
        return (TelemetryPacket packet) -> {
            toggled = tog;

            return false;
        };
    }

    public double velocity() {
        return intake.getVelocity();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Moving", powered);
        telemetry.addData("Target Velocity (tps)", targetVel * (537.6/360));
        telemetry.addData("Intake Velocity (tps)", velocity());
    }

    @Override
    public String getName() {
        return "Intake";
    }
}
