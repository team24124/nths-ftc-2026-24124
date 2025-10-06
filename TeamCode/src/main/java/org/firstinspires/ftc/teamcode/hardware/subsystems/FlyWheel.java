package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class FlyWheel implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx wheel1;
    private final DcMotorEx wheel2;
    private final Servo flap;
    public boolean powered = false;
    private double targetPower = 0;

    public FlyWheel(HardwareMap hw) {
        wheel1 = hw.get(DcMotorEx.class, "wheel1");
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wheel2 = hw.get(DcMotorEx.class, "wheel2");
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        flap = hw.get(Servo.class, "flap");
    }

    public void runFlyWheel(double distance) {
        targetPower = distance / 80; // Distance in inches / 80 (should give around max power in the small launch zone and half around the center)
        wheel1.setPower(targetPower);
        wheel2.setPower(targetPower);
        powered = true;

        // 29.5 is height of AT
        adjustFlap(distance);
    }

    public void stopFlyWheel() {
        targetPower = 0;
        wheel1.setPower(targetPower);
        wheel2.setPower(targetPower);
        powered = false;
    }

    public void adjustFlap(double distance) {
        flap.setPosition(Math.atan2(29.5, distance)/100); // / 100 to reduce to decimal
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Moving", powered);
        telemetry.addData("Power", targetPower);
        telemetry.addData("Wheel 1 ticks", wheel1.getCurrentPosition());
        telemetry.addData("Wheel 2 ticks", wheel2.getCurrentPosition());
    }

    @Override
    public String getName() {
        return "FlyWheel";
    }
}
