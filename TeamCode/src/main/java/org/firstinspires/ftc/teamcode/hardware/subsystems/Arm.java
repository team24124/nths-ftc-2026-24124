package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryObservable;

public class Arm implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx armMotor;
    private PIDF pidf;
    private final VoltageSensor voltageSensor;

    public enum State {
        PRESCORE(1),
        POSTSCORE(2),
        DEFAULT(3),
        GRAB(4);

        public final int position;

        State(int position) {
            this.position = position;
        }
    }

    private State state;

    public boolean isMoving = false;

    public Arm(HardwareMap hw) {
        pidf = new PIDF();
        pidf.setPIDF(1, 0.2, 0.2, 0.01, 0.8, 0.74);
        armMotor = hw.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        state = State.DEFAULT;
    }

    @Override
    public void periodic() {
        int position = armMotor.getCurrentPosition();
        int target = state.position;

        double power = pidf.calculate(position, target, voltageSensor.getVoltage());
        armMotor.setPower(power);
    }

    /**
     * Move the arm to a given target position as a RoadRunner action
     *
     * @param target Target position in encoder ticks for the arm to travel to
     * @return A RoadRunner Action
     */
    public Action moveTo(int target) {
        return (TelemetryPacket packet) -> {
            isMoving = true;
            int position = armMotor.getCurrentPosition();
            int tolerance = 5;

            double power = pidf.calculate(position, target, voltageSensor.getVoltage());
            armMotor.setPower(power);

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

    /**
     * Stop and reset motor. Sets motor's RunMode to RUN_USING_ENCODER after
     */
    public void stopAndResetEncoders() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Current State", getCurrentState());
        telemetry.addData("Est. Motor Position", armMotor.getCurrentPosition());
        telemetry.addData("Moving", isMoving);
    }

    public State getCurrentState(){
        return state;
    }

    @Override
    public String getName() {
        return "Arm";
    }
}
