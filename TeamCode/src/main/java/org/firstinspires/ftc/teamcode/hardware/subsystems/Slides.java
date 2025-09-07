package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryObservable;

@Config
public class Slides implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx leftSlide, rightSlide;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;

    public static PIDFCoefficients coefficients = new PIDFCoefficients(
            0.00945,
            0,
            0.0001,
            0.01
    );

    public enum State {
        HOME(0),
        HIGH_RUNG(4500),
        CLIP_HIGH_CHAMBER(8000),
        HIGH_BUCKET(10000);

        public final int position;

        State(int position) {
            this.position = position;
        }
    }

    // Create a selection array from all SlideState values
    public final ArraySelect<State> positions = new ArraySelect<>(State.values());

    public boolean isMoving = false;

    public Slides(HardwareMap hw) {
        leftSlide = hw.get(DcMotorEx.class, "left_slide");
        rightSlide = hw.get(DcMotorEx.class, "right_slide");

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");
        controller = new PIDController(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void periodic() {
        int slidePos = leftSlide.getCurrentPosition();
        int target = positions.getSelected().position;

        double p = coefficients.p, i = coefficients.i, d = coefficients.d, f = coefficients.f;

        controller.setPID(p, i, d);
        double pid = controller.calculate(slidePos, target);
        double power = (pid + f) * (12.0 / voltageSensor.getVoltage()); // Compensate for voltages
        setPower(power);
    }

    public Action setStateTo(State state) {
        positions.setSelected(state);

        return moveTo(state.position);
    }

    public Action nextPos() {
        positions.next();

        return moveTo(positions.getSelected().position);
    }

    public Action prevPos() {
        positions.previous();

        return moveTo(positions.getSelected().position);
    }

    /**
     * Move the slides to a given target position as a RoadRunner action.
     *
     * @param target Target position in encoder ticks for the slides to travel to
     * @return A RoadRunner Action
     */
    public Action moveTo(int target) {
        return (TelemetryPacket packet) -> {
            isMoving = true;
            int slidePos = leftSlide.getCurrentPosition();
            double tolerance = 0.1 * target + 10; // Check if we are within 1% of the target, with a constant of 1

            double p = coefficients.p, i = coefficients.i, d = coefficients.d, f = coefficients.f;

            // FTCDashboard Telemetry
            packet.put("Position", slidePos);
            packet.put("Target", target);
            packet.put("Position Reached?", Utilities.isBetween(slidePos, target - tolerance, target + tolerance));

            controller.setPID(p, i, d);
            double pid = controller.calculate(slidePos, target);
            double power = (pid + f + 0.4) * (12.0 / voltageSensor.getVoltage()); // Compensate for voltages
            setPower(power);

            packet.put("Power", power);

            if (Utilities.isBetween(slidePos, target - tolerance, target + tolerance)) {
                setPower(f);
                isMoving = false;
                return false; // Stop the command
            } else {
                return true; // Otherwise continue running it
            }
        };
    }

    /**
     * Set the power of both motors driving the slides given a power
     *
     * @param power Power to give to the motors
     */
    public void setPower(double power) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

    /**
     * Stop and reset all used motors. Sets motor's RunMode to RUN_WITHOUT_ENCODER after
     */
    public void stopAndResetEncoders() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Target", positions.getSelected());
        telemetry.addData("Left Slide Pos.", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide Pos.", rightSlide.getCurrentPosition());
        telemetry.addData("Is Moving", isMoving);
    }

    @Override
    public String getName() {
        return "Slides";
    }
}
