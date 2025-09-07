package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryControl;

public class Robot {
    public FlyWheel flyWheel;
    public Drivetrain driveTrain;
    public Limelight limelight;
    public ActionScheduler actions;

    public TelemetryControl telemetryControl;

    private boolean isExtended = false;
    private boolean isInScoringMode = false;

    public Robot(HardwareMap hw, Telemetry telemetry, boolean robotCentric) {
        flyWheel = new FlyWheel(hw);
        limelight = new Limelight(hw);

        if (robotCentric) {
            driveTrain = new RobotCentricDrive(hw, PoseStorage.currentPose);
        } else {
            driveTrain = new FieldCentricDrive(hw, PoseStorage.currentPose);
        }

        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl
                .subscribe(flyWheel)
                .subscribe(limelight)
                .subscribe(driveTrain)
                .subscribe(actions);
    }

    public boolean isExtended() {
        return isExtended;
    }

    public boolean isInScoringMode() {
        return isInScoringMode;
    }

    public Action xyz() {
        return new SequentialAction(
                // Collective actions
        );
    }
}