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
@TeleOp(name = "DriveOnly", group = "test")
public class DriveOnly extends OpMode {
    public static boolean robotCentric = true;
    private boolean t = true;
    private Drivetrain drivetrain;
    private ActionScheduler actions;
    private GamepadEx driver;
    private TeleOpTrajectories trajectory;
    private List<LynxModule> hubs;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new RobotCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        actions = ActionScheduler.INSTANCE;
        trajectory = TeleOpTrajectories.INSTANCE;
        actions.init();
        driver = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double y = -driver.getLeftY();
        double x = driver.getLeftX();
        double rx = -driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            actions.schedule(new InstantAction(drivetrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            actions.schedule(new InstantAction(drivetrain.getSpeeds()::next));
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            Vector2d targetPose = new Vector2d(2,2);
            trajectory.vectorAlign(drivetrain.getDrive(), targetPose);
        }

        if (t != robotCentric) {
            switchDrive();
            t = robotCentric;
        }

        driver.readButtons();

        drivetrain.drive(x, y, rx);

        drivetrain.periodic();

        ActionScheduler.INSTANCE.run();
    }
    @Override
    public void stop() {
        actions.stop();
    }
    public void switchDrive() {
        if (robotCentric) {
            drivetrain = new RobotCentricDrive(hardwareMap, drivetrain.getDrive().localizer.getPose());
        } else {
            drivetrain = new FieldCentricDrive(hardwareMap, drivetrain.getDrive().localizer.getPose());
        }
    }
}