package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;
import org.firstinspires.ftc.teamcode.util.controllers.SquID;

import java.util.List;

@Config
@TeleOp(name = "Indexer Debugger", group = "tuning")
public class IndexerDebugger extends OpMode {
    private List<LynxModule> hubs;
    private GamepadEx driver;
    private ActionScheduler actions;
    private Spindexer spindexer;
    public static double Ks, Kp, Ki, Kd = 0;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driver = new GamepadEx(gamepad1);
        actions = ActionScheduler.INSTANCE;
        actions.init();

        spindexer = new Spindexer(hardwareMap);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        spindexer.setPIDF(0.002, 0.001, 0.00006, Ks, 0.7, 0.7);
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            actions.schedule(spindexer.shootAll());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            actions.schedule(spindexer.shootNearest());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            actions.schedule(spindexer.intakeToEmpty());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            actions.schedule(spindexer.sortTo("empty"));
        }
        if (!spindexer.isMoving) {
            spindexer.periodic();
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            spindexer.states.previous();
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            spindexer.states.next();
        }

        driver.readButtons();
        actions.run();
        spindexer.updateTelemetry(telemetry);
    }
}