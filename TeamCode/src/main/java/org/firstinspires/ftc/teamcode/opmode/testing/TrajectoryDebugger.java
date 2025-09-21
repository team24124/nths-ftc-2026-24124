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
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

import java.util.List;

@Config
@TeleOp(name = "Trajectory Debugger", group = "test")
public class TrajectoryDebugger extends OpMode {
    public static boolean robotCentric = true;
    private boolean t = true;
    private Drivetrain drivetrain;
    private ActionScheduler actions;
    private GamepadEx driver;
    private TeleOpTrajectories trajectories;
    private List<LynxModule> hubs;
    private double x = 0;
    private double y = 0;
    private boolean debugX = true;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new RobotCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        actions = ActionScheduler.INSTANCE;
        trajectories = TeleOpTrajectories.INSTANCE;
        actions.init();
        driver = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double y = Math.abs(-driver.getLeftY()) > 0.05 ? -driver.getLeftY() : 0;
        double x = Math.abs(driver.getLeftX()) > 0.05 ? driver.getLeftX() : 0;
        double rx = Math.abs(-driver.getRightX()) > 0.05 ? -driver.getRightX() : 0;

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            actions.schedule(new InstantAction(drivetrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            actions.schedule(new InstantAction(drivetrain.getSpeeds()::next));
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            Pose2d targetPose = new Pose2d(x, y, drivetrain.getHeading());
            actions.schedule(trajectories.vectorAlign(drivetrain.getDrive(), targetPose));
        }

        // Switch between adjusting x and y
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            debugX = true;
        } else if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            debugX = false;
        }

        // Adjust x and y
        if (debugX) {
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                x += 1;
            } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                x -= 1;
            }
        } else {
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                y += 1;
            } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                y -= 1;
            }
        }

        // Checks if drivetrain has been switched and switches drivetrain type
        if (t != robotCentric) {
            switchDrive();
            t = robotCentric;
        }

        if (!drivetrain.getDrive().isBusy) {
            drivetrain.drive(x, y, rx, false);
        }
        drivetrain.periodic();

        driver.readButtons();

        ActionScheduler.INSTANCE.run();

        Pose2d current = drivetrain.getPosition();
        telemetry.addData("\nBusy", drivetrain.getDrive().isBusy);
        telemetry.addData("\nX", current.position.x);
        telemetry.addData("\nY", current.position.y);
        telemetry.addData("\nHeading", current.heading.toDouble());

        // Show on telemetry what is being edited
        if (debugX) {
            telemetry.addData("\n\n> Targeted X", x);
            telemetry.addData("Targeted Y", y);
        } else {
            telemetry.addData("\n\nTargeted X", x);
            telemetry.addData("> Targeted Y", y);
        }

        telemetry.update();
    }
    @Override
    public void stop() {
        actions.stop();
    }
    public void switchDrive() {
        if (robotCentric) {
            drivetrain = new RobotCentricDrive(hardwareMap, drivetrain.getPosition());
        } else {
            drivetrain = new FieldCentricDrive(hardwareMap, drivetrain.getPosition());
        }
    }
}