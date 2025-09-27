package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;


public class Spindexer implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx spindexer;
    private PIDF pidf;
    private final VoltageSensor voltageSensor;

    public enum State {
        SLOT1(1),
        SLOT2(2),
        SLOT3(3),
        IN1(1),
        IN2(2),
        IN3(3);

        public final int position;

        State(int position) {
            this.position = position;
        }
    }

    private State state;

    public boolean isMoving = false;

    // A periodic function is necessary as balls are always rolling in and moving the indexer
    public Spindexer(HardwareMap hw) {
        pidf = new PIDF();
        pidf.setPID(1, 0.2, 0.2, 0.01, 0.8);
        spindexer= hw.get(DcMotorEx.class, "arm");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        state = State.IN1;
    }

    @Override
    public void periodic() {
        int position = spindexer.getCurrentPosition();
        int target = state.position;

        double power = pidf.calculate(position, target, voltageSensor.getVoltage());
        spindexer.setPower(power);
    }

    public Action moveTo(int target) {
        return (TelemetryPacket packet) -> {
            isMoving = true;
            int position = spindexer.getCurrentPosition();
            int tolerance = 3;

            double power = pidf.calculate(position, target, voltageSensor.getVoltage());
            spindexer.setPower(power);

            if (Utilities.isBetween(position, target - tolerance, target + tolerance)) {
                isMoving = false;
                return false;
            } else {
                return true;
            }
        };
    }

    public Action moveTo(State armState) {
        state = armState;

        return moveTo(armState.position);
    }

    public void stopAndResetEncoders() {
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Current State", getCurrentState());
        telemetry.addData("Est. Motor Position", spindexer.getCurrentPosition());
        telemetry.addData("Moving", isMoving);
    }

    public State getCurrentState(){
        return state;
    }

    @Override
    public String getName() {
        return "Spindexer";
    }
}
