package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

import java.util.List;

@Config
@TeleOp(name = "FCD", group = "test")
public class FieldCentricDriveOnly extends OpMode {
    private Drivetrain drivetrain;
    private ActionScheduler actions;
    private GamepadEx driver;
    private List<LynxModule> hubs;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new FieldCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        actions = ActionScheduler.INSTANCE;
        actions.init();
        driver = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        Vector2d current = drivetrain.getDrive().localizer.getPose().position;

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            actions.schedule(new InstantAction(drivetrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            actions.schedule(new InstantAction(drivetrain.getSpeeds()::next));
        }

        // Reset pose
        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
            drivetrain.getDrive().localizer.setPose(new Pose2d(current, 0));
        }

        drivetrain.drive(x, y, rx, false);
        drivetrain.periodic();

        driver.readButtons();

        ActionScheduler.INSTANCE.run();

        telemetry.addData("\nX", current.x);
        telemetry.addData("\nY", current.y);
        telemetry.addData("\nHeading", drivetrain.getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        actions.stop();
    }
}