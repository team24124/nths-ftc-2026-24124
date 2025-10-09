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
    private double xPos = 0;
    private double yPos = 0;
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

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            actions.schedule(new InstantAction(drivetrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            actions.schedule(new InstantAction(drivetrain.getSpeeds()::next));
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            Vector2d targetPose = new Vector2d(xPos, yPos);
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
            if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                xPos += 1;
            } else if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                xPos -= 1;
            }
        } else {
            if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                yPos += 1;
            } else if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                yPos -= 1;
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
        telemetry.addData("\nX", drivetrain.getPosition().position.x);
        telemetry.addData("\nY", drivetrain.getPosition().position.y);
        telemetry.addData("\nHeading", drivetrain.getHeading());

        // Show on telemetry what is being edited
        if (debugX) {
            telemetry.addData("\n\n> Targeted X", xPos);
            telemetry.addData("Targeted Y", yPos);
        } else {
            telemetry.addData("\n\nTargeted X", xPos);
            telemetry.addData("> Targeted Y", yPos);
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