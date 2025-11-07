package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;


public class Spindexer implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx spindexer;
    private final Servo kicker;
    private final double TPR = 537.6;
    private PIDF pd;
    private final VoltageSensor voltageSensor;

    public enum State {
        SLOT1(264), // Slots are shoot positions
        SLOT2(90), // Slots increase CCW, IN1 is referencing the same slot as SLOT1
        SLOT3(448),
        IN1(0), // Ins are slots facing towards the intake
        IN2(358), // Ins increase CCW. Since moving CCW results in a positive increase, the second slot CCW from the first one is on the rear left of the robot, making its position -178 + 537.6
        IN3(179);

        public final int position;

        State(int position) {
            this.position = position;
        }
    }

    public final ArraySelect<State> states = new ArraySelect<>(State.values());
    public List<String> slots = new ArrayList<>(Arrays.asList("empty", "empty", "empty"));

    public boolean isMoving;

    public Spindexer(HardwareMap hw) {
        pd = new PIDF();
        pd.setPD(0, 0, 0);
        spindexer = hw.get(DcMotorEx.class, "spindexer");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        kicker = hw.get(Servo.class, "kicker");
        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        states.setSelected(State.IN1);
    }

    // A periodic function is necessary as balls are always rolling in and shifting the indexer
    @Override
    public void periodic() {
        double target = states.getSelected().position;
        double position = spindexer.getCurrentPosition() % TPR; // Normalize to [0, 537.6)
        if (position < 0) position += TPR;

        // Compute raw difference
        double error = target - position;

        // Wrap error into (-TPR/2, +TPR/2]
        if (error > TPR / 2) error -= TPR;
        else if (error < -TPR / 2) error += TPR;

        double adjustedPosition = target - error;

        double power = pd.calculate(adjustedPosition, target, voltageSensor.getVoltage());
        spindexer.setPower(power);
    }

    public Action autonPeriodic() {
        return (TelemetryPacket packet) -> {
            periodic();

            return true;
        };
    }

    private Action moveToState() {
        return (TelemetryPacket packet) -> {
            double target = states.getSelected().position;
            double position = spindexer.getCurrentPosition() % TPR; // Normalize to [0, 537.6)
            if (position < 0) position += TPR;

            // Compute raw difference
            double error = target - position;

            // Wrap error into (-TPR/2, +TPR/2]
            if (error > TPR / 2) error -= TPR;
            else if (error < -TPR / 2) error += TPR;

            double adjustedPosition = target - error;

            double power = pd.calculate(adjustedPosition, target, voltageSensor.getVoltage());
            spindexer.setPower(power);

            isMoving = !Utilities.isBetween(getPower(), -0.05, 0.05);
            return !isMoving;
        };
    }

    // Sort to the first specified colour in the array of slots
    public Action sortTo(String colour) {
        int firstColour = slots.indexOf(colour);
        if (firstColour != -1 && states.getSelectedIndex() != firstColour) {
            states.moveSelection(firstColour - states.getSelectedIndex());
        }

        return moveToState();
    }

    public Action sortTo(int slot) {
        states.setSelected(slot);

        return moveToState();
    }

    public Action intakeToEmpty() {
        int firstEmpty = slots.indexOf("empty") + 3;
        if (firstEmpty != 2 && states.getSelectedIndex() != firstEmpty) {
            states.moveSelection(firstEmpty - states.getSelectedIndex());
        }

        return moveToState();
    }

    public Action shootOne() {
        ElapsedTime timer = new ElapsedTime();
        kicker.setPosition(0.3);

        return (TelemetryPacket packet) -> {
            if (timer.seconds() < 0.3) {
                return true;
            }

            kicker.setPosition(0);
            return timer.seconds() < 0.4;
        };
    }

    public Action shootAllNoSort() {
        return new SequentialAction(
                sortTo(3),
                shootOne(),
                sortTo(4),
                shootOne(),
                sortTo(5),
                shootOne(),
                intakeToEmpty()
        );
    }

    public void stopAndResetEncoders() {
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getPower() {
        return spindexer.getPower();
    }

    public void setPD(double Kp, double Kd, double sf) {
        pd.setPD(Kp, Kd, sf);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        int target = states.getSelected().position;
        double position = spindexer.getCurrentPosition() % TPR;
        position = (position < 0) ? position + TPR : position;
        double error = target - position;
        if (error > TPR / 2) error -= TPR;
        else if (error < -TPR / 2) error += TPR;
        double adjustedPosition = target - error;

        telemetry.addData("Current State", getCurrentState());
        telemetry.addData("Est. Motor Position", position);
        telemetry.addData("Adjusted Position", adjustedPosition);
        telemetry.addData("Est. target",  target);
        telemetry.addData("Moving", isMoving);
    }

    public State getCurrentState(){
        return states.getSelected();
    }

    @Override
    public String getName() {
        return "Spindexer";
    }
}