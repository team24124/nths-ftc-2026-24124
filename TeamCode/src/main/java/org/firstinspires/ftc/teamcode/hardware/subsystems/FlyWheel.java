package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class FlyWheel implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx flyWheel;
    private boolean powered = false;
    private double targetPower = 0;

    public FlyWheel(HardwareMap hw) {
        flyWheel = hw.get(DcMotorEx.class, "arm");
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runFlyWheel(double distance) {
        targetPower = distance * 0.01; // Many ways to tune (mechanism not yet determined)
        flyWheel.setPower(targetPower);
        powered = true;
    }

    public void stopFlyWheel() {
        targetPower = 0;
        flyWheel.setPower(targetPower);
        powered = false;
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Moving", powered);
        telemetry.addData("Power", targetPower);
    }

    @Override
    public String getName() {
        return "FlyWheel";
    }
}
