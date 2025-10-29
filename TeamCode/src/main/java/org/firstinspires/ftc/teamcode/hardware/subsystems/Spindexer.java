package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class Spindexer implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx spindexer;
    private final double TPR = 537.6;
    private PIDF pd;
    private final VoltageSensor voltageSensor;

    public enum State {
        SLOT1(329), // Slots are compressed ball positions (shoot positions)
        SLOT2(155), // Slots increase CCW, IN1 is referencing the same slot as SLOT1
        SLOT3(513), // Slots are moved 65 extra tick units to ensure compression
        IN1(0), // Ins are slots facing towards the intake (sort and intake positions)
        IN2(358), // Ins increase CCW. Since moving CCW results in a positive increase, the second slot CCW from the first one is on the bottom left of the robot, making its position -178 + 537.6
        IN3(179);

        public final int position;

        State(int position) {
            this.position = position;
        }
    }

    public final ArraySelect<State> states = new ArraySelect<>(State.values());
    public List<String> slots = new ArrayList<>(Arrays.asList("empty", "empty", "empty"));

    public boolean isMoving = false;

    public Spindexer(HardwareMap hw) {
        pd = new PIDF();
        pd.setPD(0, 0, 0);
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

            // Reconstruct adjusted position for PIDF (if it takes position, not error)
            double adjustedPosition = target - error;

            double power = pd.calculate(adjustedPosition, target, voltageSensor.getVoltage());
            spindexer.setPower(power);
        }
    }

    public Action autonPeriodic() {
        return (TelemetryPacket packet) -> {
            periodic();

            return true;
        };
    }

    private Action outputTo(State state) {
        int target = state.position;

        return (TelemetryPacket packet) -> {
            isMoving = true;
            double position = spindexer.getCurrentPosition() % TPR; // Normalize to 537.6 ticks (one rotation)
            position = (position < 0) ? position + TPR : position; // Eliminate negatives

            if (target < position) {
                position -= TPR;
            }

            int tolerance = 10;

            double power = pd.calculate(position, target, voltageSensor.getVoltage());
            spindexer.setPower(power);

            if (Utilities.isBetween(position % TPR, target - tolerance, target + tolerance)) {
                isMoving = false;
                return false;
            } else {
                return true;
            }
        };
    }

    private Action sortTo(State state) {
        int target = state.position;

        return (TelemetryPacket packet) -> {
            isMoving = true;
            double position = spindexer.getCurrentPosition() % TPR;
            position = (position < 0) ? position + TPR : position;

            if (target > position) {
                position += TPR;
            }

            int tolerance = 10;

            double power = pd.calculate(position, target, voltageSensor.getVoltage());
            spindexer.setPower(power);

            if (Utilities.isBetween(position % TPR, target - tolerance, target + tolerance)) {
                isMoving = false;
                return false;
            } else {
                return true;
            }
        };
    }

    // Sort to the first specified colour in the array of slots
    public Action sortTo(String colour) {
        int firstColour = slots.indexOf(colour) + 2;
        if (firstColour == 1 || states.getSelectedIndex() == ((firstColour) % 3) + 3) {
            return (TelemetryPacket packet) -> false;
        }

        states.moveSelection((((firstColour) % 3) + 3) - states.getSelectedIndex());
        return sortTo(states.getSelected());
    }

    // Shoot 1 ball
    public Action shootNearest() {
        states.moveSelection((((states.getSelectedIndex() + 5) % 3) + 3) - states.getSelectedIndex());

        return outputTo(states.getSelected());
    }

    // Shoot all indexes
    public Action shootAll() {
        return new SequentialAction(
                outputTo(states.moveSelection((((states.getSelectedIndex() + 1) % 3) + 3) - states.getSelectedIndex()).getSelected()),
                outputTo(states.moveSelection((((states.getSelectedIndex() + 1) % 3) + 3) - states.getSelectedIndex()).getSelected()),
                outputTo(states.moveSelection((((states.getSelectedIndex() + 1) % 3) + 3) - states.getSelectedIndex()).getSelected())
        );
    }

    public Action intakeToEmpty() {
        int firstEmpty = slots.indexOf("empty") + 3; // From 0, 1, 2 to 3, 4, 5 which are the indexes of the IN slots
        if (firstEmpty == 2 || states.getSelectedIndex() == firstEmpty) {
            return (TelemetryPacket packet) -> false;
        }

        states.moveSelection(firstEmpty - states.getSelectedIndex());
        return sortTo(states.getSelected());
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

        // Compute raw difference
        double error = target - position;

        // Wrap error into (-TPR/2, +TPR/2]
        if (error > TPR / 2) error -= TPR;
        else if (error < -TPR / 2) error += TPR;

        // Reconstruct adjusted position for PIDF (if it takes position, not error)
        double adjustedPosition = target - error;

        telemetry.addData("Current State", getCurrentState());
        telemetry.addData("Est. Motor Position", position);
        telemetry.addData("Est. Motor Position2", adjustedPosition);
        telemetry.addData("Est. target", target - adjustedPosition);
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