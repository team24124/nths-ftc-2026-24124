package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.util.plotting.Oscillator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class Spindexer implements SubsystemBase, TelemetryObservable {
    public final DcMotorEx spindexer;
    public final Servo kicker;
    private final double TPR = 537.6;
    public PIDF pd;
    public final VoltageSensor voltageSensor;
    public Oscillator os;

    public enum State {
        SLOT1(267), // Slots are shoot positions
        SLOT2(90), // Slots increase CCW, IN1 is referencing the same slot as SLOT1
        SLOT3(449),
        IN1(0), // Ins are slots facing towards the intake
        IN2(358), // Ins increase CCW. Since moving CCW results in a positive increase, the second slot CCW from the first one is on the rear left of the robot, making its position -178 + 537.6
        IN3(179);

        public final int position;

        State(int position) {
            this.position = position;
        }
    }

    public final ArraySelect<State> states = new ArraySelect<>(State.values());
    public List<String> slots = new ArrayList<>(Arrays.asList("green", "purple", "purple"));

    public boolean isMoving;
    public boolean autoMoving;
    public boolean kickScheduled = false;
    public int shotCount = 0;

    public Spindexer(HardwareMap hw) {
        os = new Oscillator(new Double[]{-20.0, 20.0}, 0.2);
        os.enableOscillation(true);
        pd = new PIDF();
        pd.setPD(0.0035, 0.000001, 0.7);
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
        if (!isMoving && states.getSelectedIndex() > 2) {
            target -= os.returnSetpoint();
        }
        double position = spindexer.getCurrentPosition() % TPR; // Normalize to [0, 537.6)
        if (position < 0) position += TPR;

        // Compute raw difference
        double error = target - position;

        // Wrap error into (-TPR/2, +TPR/2]
        if (error > TPR / 2) error -= TPR;
        else if (error < -TPR / 2) error += TPR;

        double adjustedPosition = target - error;

        autoMoving = !(Utilities.isBetween(position, target - 20, target + 20) || (states.getSelected() == State.IN1 && Utilities.isBetween(position, 537.6 - 20, 537.6)));
        double power = pd.calculate(adjustedPosition, target, voltageSensor.getVoltage());

        if (!autoMoving) {
            spindexer.setPower(0);
        } else {
            spindexer.setPower(power);
        }
    }

    public Action autonPeriodic() {
        return (TelemetryPacket packet) -> {
            periodic();

            return true;
        };
    }

    public Action moveToState() {
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

            if (Utilities.isBetween(position, target - 20, target + 20) || (states.getSelected() == State.IN1 && Utilities.isBetween(position, 537.6 - 20, 537.6))) {
                isMoving = kickScheduled;
                return false;
            } else {
                isMoving = true;
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

        return moveToState();
    }

    public Action sortTo(int slot) {
        states.setSelected(slot);

        return moveToState();
    }

    public Action sortTo(String colour) {
        int firstColour = slots.indexOf(colour);
        if (firstColour != -1 && states.getSelectedIndex() != firstColour) {
            states.moveSelection(firstColour - states.getSelectedIndex());
        }

        return moveToState();
    }

    public Action intakeToEmpty() {
        int firstEmpty = slots.indexOf("empty") + 3;
        if (firstEmpty != 2 && states.getSelectedIndex() != firstEmpty) {
            states.moveSelection(firstEmpty - states.getSelectedIndex());
        }

        return moveToState();
    }

    public Action kick() {
        ElapsedTime timer = new ElapsedTime();

        return (TelemetryPacket packet) -> {
            if (timer.seconds() < 0.18) {
                return true;
            }
            kicker.setPosition(0.35);

            if (timer.seconds() < 0.77) {
                return true;
            }

            kicker.setPosition(0.76);
            if (timer.seconds() < 1.05) {
                return true;
            }
            kickScheduled = false;
            isMoving = false;
            return false;
        };
    }

    public Action shootOne() {
        if (!isMoving) {
            shotCount += 1;
            if (shotCount == 3) {
                shotCount = 0;
            }
            slots.remove(shotCount);
            slots.add(shotCount, "empty");
            kickScheduled = true;
            return new SequentialAction(
                    sortTo(shotCount),
                    kick()
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
        telemetry.addData("Slots",  slots.toString());
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