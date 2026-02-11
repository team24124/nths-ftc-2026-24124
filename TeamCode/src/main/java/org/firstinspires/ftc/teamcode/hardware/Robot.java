package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.teamcode.util.Utilities;

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
    private double TPR = 806.4;

    public Robot(HardwareMap hw, Telemetry telemetry, boolean robotCentric) {
        flywheel = new Flywheel(hw);
        intake = new Intake(hw);
        spindexer = new Spindexer(hw);
        limelight = new Limelight(hw);
        colorSensor = new REVColourV3(hw);

        if (robotCentric) {
            drivetrain = new RobotCentricDrive(hw, PoseStorage.currentPose);
        } else {
            drivetrain = new FieldCentricDrive(hw, PoseStorage.currentPose);
        }

        actions = ActionScheduler.INSTANCE;
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

    public Action intakeAutoPeriodic() {
        return (TelemetryPacket packet) -> {
            if (intake.overridden) {
                return false;
            } else {
                if (intake.toggled) {
                    if (!spindexer.slots.contains("empty")) {
                        intake.targetVel = 0; // Changed from 0
                        intake.intake.setVelocity(0);
                        intake.powered = false;
                        return false;
                    }
                    if (spindexer.isAutoMoving) {
                        intake.targetVel = (-((double) 312 / 60) * 175) * (TPR / 360);
                        intake.intake.setVelocity(0);
                        intake.powered = false;
                        return true;
                    } else {
                        if (spindexer.states.getSelectedIndex() > 2 && Objects.equals(spindexer.slots.get(spindexer.states.getSelectedIndex() - 3), "empty")) {
                            spindexer.slots.remove(spindexer.states.getSelectedIndex() - 3);
                            spindexer.slots.add(spindexer.states.getSelectedIndex() - 3, colorSensor.getColour());
                        }
                        intake.targetVel = -((double) 312 / 60) * 360;
                        intake.intake.setVelocity(intake.targetVel * (TPR / 360));
                        intake.powered = true;

                        int firstEmpty = spindexer.slots.indexOf("empty") + 3;
                        if (firstEmpty != 2 && spindexer.states.getSelectedIndex() != firstEmpty) {
                            spindexer.states.moveSelection(firstEmpty - spindexer.states.getSelectedIndex());
                        }
                    }
                } else {
                    intake.targetVel = 0; // Changed from 0
                    intake.intake.setVelocity(0);
                    intake.powered = false;
                }
                return true;
            }
        };
    }

    public Action intakePeriodic() {
        if (intake.toggled && !spindexer.isMoving && spindexer.slots.contains("empty")) {
            if (spindexer.states.getSelectedIndex() > 2 && Objects.equals(spindexer.slots.get(spindexer.states.getSelectedIndex() - 3), "empty")) {
                spindexer.slots.remove(spindexer.states.getSelectedIndex() - 3);
                spindexer.slots.add(spindexer.states.getSelectedIndex() - 3, colorSensor.getColour());
            }
            if (spindexer.states.getSelectedIndex() < 3 || !Objects.equals(spindexer.slots.get(spindexer.states.getSelectedIndex() - 3), "empty")) {
                return spindexer.inTo("empty");
            } else {
                return intake.runIntake();
            }
        } else {
            return intake.stopIntake();
        }
    }

    public void removeIndexed(int i) {
        spindexer.slots.remove(i);
        spindexer.slots.add(i, "empty");
    }
}