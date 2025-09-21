package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretBase;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;

public class Robot {
    public FlyWheel flyWheel;

    public TurretBase turretBase;

    public Drivetrain driveTrain;

    public Limelight limelight;

    public ActionScheduler actions;

    public TelemetryControl telemetryControl;

    public Robot(HardwareMap hw, Telemetry telemetry, boolean robotCentric) {
        flyWheel = new FlyWheel(hw);
        turretBase = new TurretBase(hw);
        limelight = new Limelight(hw);

        if (robotCentric) {
            driveTrain = new RobotCentricDrive(hw, PoseStorage.currentPose);
        } else {
            driveTrain = new FieldCentricDrive(hw, PoseStorage.currentPose);
        }

        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl
                .subscribe(flyWheel)
                .subscribe(turretBase)
                .subscribe(limelight)
                .subscribe(driveTrain)
                .subscribe(actions); // Display on telemetry
    }

    public Action xyz() {
        return new SequentialAction(
                // Collective actions
        );
    }
}