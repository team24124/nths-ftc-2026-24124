package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.hardware.subsystems.REVColourV3;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;

import java.util.List;
import java.util.Objects;

public class Robot {
    public Flywheel flywheel;
    public Intake intake;
    public Spindexer spindexer;

    public Drivetrain drivetrain;
    public Limelight limelight;
    public REVColourV3 colorSensor;

    public ActionScheduler actions;
    public TelemetryControl telemetryControl;

    public Robot(HardwareMap hw, Telemetry telemetry, boolean robotCentric) {
        flywheel = new Flywheel(hw);
        intake = new Intake(hw);
        spindexer = new Spindexer(hw);
        limelight = new Limelight(hw);

        if (robotCentric) {
            drivetrain = new RobotCentricDrive(hw, PoseStorage.currentPose);
        } else {
            drivetrain = new FieldCentricDrive(hw, PoseStorage.currentPose);
        }

        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl
                .subscribe(flywheel)
                .subscribe(intake)
                .subscribe(spindexer)
                .subscribe(limelight)
                .subscribe(drivetrain)
                .subscribe(colorSensor)
                .subscribe(actions); // Display on telemetry
    }

    public Action intakePeriodic() {
        if (intake.toggled) {
            if (spindexer.isMoving || !spindexer.slots.contains("empty")) {
                return intake.stopIntake();
            } else {
                if (spindexer.states.getSelectedIndex() > 2 && Objects.equals(spindexer.slots.get(spindexer.states.getSelectedIndex() - 3), "empty")) {
                    spindexer.slots.remove(spindexer.states.getSelectedIndex() - 3);
                    spindexer.slots.add(spindexer.states.getSelectedIndex() - 3, colorSensor.getColour());
                }
                return new SequentialAction(
                        spindexer.intakeToEmpty(),
                        intake.runIntake()
                );
            }
        } else {
            return (TelemetryPacket packet) -> false;
        }
    }

    public Action shootColor(String str) {
        return new SequentialAction(
                spindexer.sortTo(spindexer.slots.indexOf(str)),
                spindexer.kick(),
                removeIndexed()
        );
    }

    public Action orderedShot(List<String> pattern) {
        while (!flywheel.primed) {

        }
        return spindexer.shootAll(pattern);
    }

    public Action removeIndexed() {
        return (TelemetryPacket packet) -> {
            spindexer.slots.remove(spindexer.states.getSelectedIndex());
            spindexer.slots.add(spindexer.states.getSelectedIndex(), "empty");

            return false;
        };
    }
}