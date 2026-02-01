package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;


public class Spindexer implements SubsystemBase, TelemetryObservable {
    public final DcMotorEx spindexer;
    private final double TPR = 806.4;
    public PIDF pd;
    public final VoltageSensor voltageSensor;
    private boolean distanceFar = false;

    public enum State {
        // Previously 230, 48, 410, 0, 358, 179
        SLOT1(345), // Slots are shoot positions
        SLOT2(72), // Slots increase CCW, IN1 is referencing the same slot as SLOT1
        SLOT3(615),
        IN1(0), // Ins are slots facing towards the intake
        IN2(537), // Ins increase CCW. Since moving CCW results in a positive increase, the second slot CCW from the first one is on the rear left of the robot, making its position -178 + 537.6
        IN3(268);

        public final int position;

        State(int position) {
            this.position = position;
        }
    }

    public final ArraySelect<State> states = new ArraySelect<>(State.values());
    public List<String> slots = new ArrayList<>(Arrays.asList("green", "purple", "purple"));
    //public List<String> slots = new ArrayList<>(Arrays.asList("empty", "empty", "empty"));

    public boolean isMoving;
    public boolean isAutoMoving = false;
    private double prevStatePos = 0;

    public Spindexer(HardwareMap hw) {
        pd = new PIDF();
        pd.setPD(0.004, 0.000012, 0.3);
        spindexer = hw.get(DcMotorEx.class, "spindexer");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        states.setSelected(State.IN1);
    }

    // A periodic function is necessary as balls are always rolling in and shifting the indexer
    @Override
    public void periodic() {
        if (!isMoving) {
            double target = states.getSelected().position;
            double position = spindexer.getCurrentPosition() % TPR; // Normalize to [0, 537.6)
            if (position < 0) position += TPR;

            // Compute raw difference
            double error = target - position;

            // Wrap error into (-TPR/2, +TPR/2]
            if (error > TPR / 2) error -= TPR;
            else if (error < -TPR / 2) error += TPR;

            double adjustedPosition = target - error;

            double power = Range.clip(pd.calculate(adjustedPosition, target, voltageSensor.getVoltage()), -0.5, 0.5);
            spindexer.setPower(power);

            if (Utilities.isBetween(position, target - 27, target + 27) || (states.getSelected() == State.IN1 && Utilities.isBetween(position, TPR - 27, TPR))) {
                isAutoMoving = false;
            } else {
                isAutoMoving = true;
            }
        }
    }

    public Action autonPeriodic() {
        return (TelemetryPacket packet) -> {
            periodic();

            return true;
        };
    }

    // Commented - used only with a passive indexer
    private Action outputToState() {
        AtomicBoolean inInitialLoop = new AtomicBoolean(true);
        return (TelemetryPacket packet) -> {
            isMoving = true;
            double target = states.getSelected().position;
            double position = spindexer.getCurrentPosition() % TPR; // Normalize to [0, 537.6) (one rotation)
            if (position < 0) position += TPR; // Eliminate negatives

            if (inInitialLoop.get() && prevStatePos == target && target > position) {
                position = target + 10;
            } else {
                inInitialLoop.set(false);
            }

            // Wrap around
            if (target < position) {
                position -= TPR;
            }

            int tolerance = 35;

            double power;
            if (distanceFar) {
                power = Range.clip(pd.calculate(position, target, voltageSensor.getVoltage()), -0.25, 0.25);
            } else {
                power = Range.clip(pd.calculate(position, target, voltageSensor.getVoltage()), -0.34, 0.34);
            }
            spindexer.setPower(power);

            if (Utilities.isBetween(position % TPR, target - tolerance, target + tolerance) || (states.getSelected() == State.IN1 && Utilities.isBetween(position, TPR - tolerance, TPR))) {
                isMoving = false;
                return false;
            } else {
                return true;
            }
        };
    }

    private Action sortToState() {
        return (TelemetryPacket packet) -> {
            isMoving = true;
            double target = states.getSelected().position;
            double position = spindexer.getCurrentPosition() % TPR;
            if (position < 0) position += TPR;

            // Wrap around
            if (target > position) {
                position += TPR;
            }

            int tolerance = 35;

            double power = Range.clip(pd.calculate(position, target, voltageSensor.getVoltage()), -0.3, 0.3);
            spindexer.setPower(power);

            if (Utilities.isBetween(position % TPR, target - tolerance, target + tolerance) || (states.getSelected() == State.IN1 && Utilities.isBetween(position, TPR - tolerance, TPR))) {
                isMoving = false;
                return false;
            } else {
                return true;
            }
        };
    }


    // Sort to the first specified colour in the array of slots
    public Action inTo(String colour) {
        int firstColour = slots.indexOf(colour) + 3;
        if (firstColour != 2 && states.getSelectedIndex() != firstColour) {
            states.moveSelection(firstColour - states.getSelectedIndex());
        }

        return sortToState();
    }

    public Action sortTo(int slot) {
        if (!isMoving) {
            states.setSelected(slot);

            return sortToState();
        } else {
            return (TelemetryPacket p) -> false;
        }
    }

    public Action outputTo(int slot) {
        prevStatePos = states.getSelected().position;
        states.setSelected(slot);

        return outputToState();
    }

    // Meant to shoot from IN1 to SLOT1
    public Action shootAll() {
        if (!isMoving) {
            return new SequentialAction(
                    outputTo(0),
                    removeIndexed(0),
                    removeIndexed(1),
                    removeIndexed(2)
            );
        } else {
            return (TelemetryPacket packet) -> false;
        }
    }

    public Action removeIndexed() {
        return (TelemetryPacket packet) -> {
            slots.remove(states.getSelectedIndex());
            slots.add(states.getSelectedIndex(), "empty");

            return false;
        };
    }

    public Action removeAllIndexed() {
        return (TelemetryPacket packet) -> {
            slots.remove(0);
            slots.add(0, "empty");
            slots.remove(1);
            slots.add(1, "empty");
            slots.remove(2);
            slots.add(2, "empty");

            return false;
        };
    }

    public Action removeIndexed(int i) {
        return (TelemetryPacket packet) -> {
            slots.remove(i);
            slots.add(i, "empty");

            return false;
        };
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

    public void updateDistance(double distance) {
        distanceFar = (distance > 120);
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

        telemetry.addData("Spindexer Current State", getCurrentState());
        telemetry.addData("Spindexer Motor Position", position);
        telemetry.addData("Spindexer Adjusted Position", adjustedPosition);
        telemetry.addData("Spindexer Target",  target);
        telemetry.addData("Spindexer Slots",  slots.toString());
        telemetry.addData("Spindexer Moving", isMoving);
    }

    public State getCurrentState(){
        return states.getSelected();
    }

    @Override
    public String getName() {
        return "Spindexer";
    }
}