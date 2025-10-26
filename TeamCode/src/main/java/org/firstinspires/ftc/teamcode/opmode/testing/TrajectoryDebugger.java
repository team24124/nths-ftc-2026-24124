package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
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
@TeleOp(name = "Trajectories", group = "test")
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
    private double heading = 0;

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
            Vector2d targetPose = new Vector2d(xPos, yPos);
            actions.schedule(trajectories.vectorAlign(drivetrain.getDrive(), targetPose));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            Pose2d targetPose = new Pose2d(xPos, yPos, heading);
            actions.schedule(trajectories.poseAlign(drivetrain.getDrive(), targetPose));
        }

        // Adjust x and y
        if (driver.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            yPos -= 0.08;
        } else if (driver.isDown(GamepadKeys.Button.DPAD_LEFT)) {
            yPos += 0.08;
        }
        if (driver.isDown(GamepadKeys.Button.DPAD_UP)) {
            xPos += 0.08;
        } else if (driver.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            xPos -= 0.08;
        }
        if (driver.isDown(GamepadKeys.Button.Y)) {
            heading += 0.04;
        } else if (driver.isDown(GamepadKeys.Button.A)) {
            heading -= 0.04;
        }

        // Checks if drivetrain has been switched and switches drivetrain type
        if (driver.wasJustPressed(GamepadKeys.Button.B) && t != robotCentric) {
            switchDrive();
            t = robotCentric;
        }

        if (!drivetrain.getDrive().isBusy) {
            drivetrain.drive(x, y, rx, false);
        }
        drivetrain.periodic();

        driver.readButtons();

        ActionScheduler.INSTANCE.run();

        telemetry.addData("\nBusy", drivetrain.getDrive().isBusy);
        telemetry.addData("\nX", drivetrain.getPosition().position.x);
        telemetry.addData("\nY", drivetrain.getPosition().position.y);
        telemetry.addData("\nHeading", drivetrain.getHeading());

        telemetry.addData("\n\nTargeted X", xPos);
        telemetry.addData("Targeted Y", yPos);
        telemetry.addData("Targeted Heading", heading);

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