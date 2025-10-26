package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;
import org.firstinspires.ftc.teamcode.util.controllers.SquID;

import java.util.List;

@Config
@TeleOp(name = "Intake Debugger", group = "test")
public class IntakeDebugger extends OpMode {
    private List<LynxModule> hubs;
    private GamepadEx driver;
    private ActionScheduler actions;
    private Intake intake;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        actions = ActionScheduler.INSTANCE;
        actions.init();

        driver = new GamepadEx(gamepad1);

        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            actions.schedule(intake.runIntake());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            actions.schedule(intake.stopIntake());
        }

        actions.run();
        driver.readButtons();
        intake.periodic();

        telemetry.addData("vel", intake.velocity());
        telemetry.update();
    }
}