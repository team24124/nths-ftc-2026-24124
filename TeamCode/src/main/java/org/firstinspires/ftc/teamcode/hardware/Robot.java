package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slides;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryControl;

public class Robot {
    public Arm arm;
    public FlyWheel flyWheel;
    public Claw claw;
    public Drivetrain driveTrain;
    public Limelight limelight;
    public Slides slides;
    
    public ActionScheduler actions;

    public TelemetryControl telemetryControl;

    private boolean isExtended = false;
    private boolean isInScoringMode = false;

    public Robot(HardwareMap hw, Telemetry telemetry, boolean robotCentric) {
        arm = new Arm(hw);
        flyWheel = new FlyWheel(hw);
        claw = new Claw(hw);
        slides = new Slides(hw);
        limelight = new Limelight(hw);

        if (robotCentric) {
            driveTrain = new RobotCentricDrive(hw, PoseStorage.currentPose);
        } else {
            driveTrain = new FieldCentricDrive(hw, PoseStorage.currentPose);
        }

        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl
                .subscribe(arm)
                .subscribe(flyWheel)
                .subscribe(claw)
                .subscribe(slides)
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

    public Action extendCollection() {
        isExtended = true;
        return new SequentialAction(
                new ParallelAction(
                        arm.moveTo(Arm.State.GRAB),
                        claw.setElbowPosition(Claw.ElbowState.HOVER),
                        claw.setPivotPosition(Claw.PivotState.ONEEIGHTY)
                )
        );
    }

    public Action retractCollection() {
        isExtended = false;
        return new SequentialAction(
                claw.setElbowPosition(Claw.ElbowState.GRAB),
                claw.setClawPosition(Claw.ClawState.CLOSED),
                new SleepAction(0.1),
                new ParallelAction(
                        claw.setElbowPosition(Claw.ElbowState.PASSTHROUGH)
                ),
                new SleepAction(1),
                claw.setPivotPosition(Claw.PivotState.TWOSEVENTY)
        );
    }

    public Action collectFromWall() {
        isInScoringMode = false;
        return new SequentialAction(
                arm.moveTo(Arm.State.GRAB)
        );
    }

    public Action resetControlArm() {
        return new SequentialAction(
                arm.moveTo(Arm.State.DEFAULT)
        );
    }
}