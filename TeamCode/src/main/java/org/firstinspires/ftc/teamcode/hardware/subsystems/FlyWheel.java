package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

public class FlyWheel implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx wheel1;
    private final DcMotorEx wheel2;
    private final Servo flap;
    public boolean powered = false;
    private double targetVel = 0;
    private PIDF velPD = new PIDF();

    public FlyWheel(HardwareMap hw) {
        wheel1 = hw.get(DcMotorEx.class, "wheel1");
        wheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wheel2 = hw.get(DcMotorEx.class, "wheel2");
        wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        flap = hw.get(Servo.class, "flap");

        velPD.setPD(0, 0, 0); // PD chosen due to potential accumulation of integral while moving and adjusting power w distance
    }

    @Override
    public void periodic(){
        power(targetVel);
    }

    public Action runFlyWheel(double distance) {
        return (TelemetryPacket packet) -> {
            //targetVel = math; // Distance in inches / 80 (should give around max power in the small launch zone and half around the center)
            powered = true;

            // 29.5 is height of AT
            adjustFlap(distance);

            return false;
        };
    }

    public Action stopFlyWheel() {
        return (TelemetryPacket packet) -> {
            targetVel = 0;
            powered = false;

            return false;
        };
    }

    public Action autonPeriodic() {
        return (TelemetryPacket packet) -> {
            periodic();

            return true;
        };
    }

    public void adjustFlap(double distance) {
        flap.setPosition(Math.atan2(29.5, distance)/100); // / 100 to reduce to decimal
    }

    private void power(double targetPower) {
        wheel1.setPower(targetPower);
        wheel2.setPower(targetPower);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Moving", powered);
        telemetry.addData("Target Velocity", targetVel);
        telemetry.addData("Wheel 1 Velocity", wheel1.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Wheel 2 Velocity", wheel2.getVelocity(AngleUnit.DEGREES));
    }

    @Override
    public String getName() {
        return "FlyWheel";
    }
}
