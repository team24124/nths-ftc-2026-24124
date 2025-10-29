package org.firstinspires.ftc.teamcode.opmode.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

import java.util.List;

@TeleOp(name = "Intake", group = "test")
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

        driver = new GamepadEx(gamepad1);
        actions = ActionScheduler.INSTANCE;
        actions.init();

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

        driver.readButtons();
        actions.run();

        telemetry.addData("Velocity", intake.velocity());
        telemetry.update();
    }
}